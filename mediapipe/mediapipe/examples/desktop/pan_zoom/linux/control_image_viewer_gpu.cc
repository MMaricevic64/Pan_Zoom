// Copyright 2019 The MediaPipe Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// An example of sending OpenCV webcam frames into a MediaPipe graph.
// This example requires a linux computer and a GPU with EGL support drivers.
#include <cstdlib>

/* socket related */
#include <sys/socket.h>
#include <unistd.h>
#include <sys/un.h>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/opencv_highgui_inc.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"
#include "mediapipe/gpu/gl_calculator_helper.h"
#include "mediapipe/gpu/gpu_buffer.h"
#include "mediapipe/gpu/gpu_shared_data_internal.h"

//Take stream from /mediapipe/graphs/hand_tracking/hand_detection_desktop_live.pbtxt
// RendererSubgraph - LANDMARKS:hand_landmarks
#include "mediapipe/calculators/util/landmarks_to_render_data_calculator.pb.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/formats/classification.pb.h"


constexpr char kInputStream[] = "input_video";
constexpr char kOutputStream[] = "output_video";
constexpr char kWindowName[] = "MediaPipe";
constexpr char kLandmarksStream[] = "hand_landmarks";
constexpr char kHandedness[] = "handedness";

ABSL_FLAG(std::string, calculator_graph_config_file, "",
          "Name of file containing text format CalculatorGraphConfig proto.");
ABSL_FLAG(std::string, input_video_path, "",
          "Full path of video to load. "
          "If not provided, attempt to use a webcam.");
ABSL_FLAG(std::string, output_video_path, "",
          "Full path of where to save result (.mp4 only). "
          "If not provided, show result in a window.");

std::pair<float, float> normalized_to_pixel_coordinates(
    float normalized_x, float normalized_y, int image_width, int image_height) {
  int x_px = std::min((int)std::floor(normalized_x * image_width), image_width - 1);
  int y_px = std::min((int)std::floor(normalized_y * image_height), image_height - 1);
  return {x_px, y_px};
}

float distance(float x1, float y1, float x2, float y2) {
  float dx = x2 - x1;
  float dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

enum class MessageType {
  None,
  Move = 1,
  Zoom,
};

enum PanState {
  Start = 1,
  Active,
  Stop
};

enum class Gesture {
  None,
  Zoom,
  Pan,
};

enum Handedness {
  None,
  Left,
  Right
};

/* message struct for sending to the image-viewer over socket */
struct GestureMessage {
  MessageType type;
  float dx;
  float dy;
  bool zoom_in; /* zoom-in or zoom-out? */
  unsigned PanState;
};

struct HandState {
  /* thumb tip coordinates (normalized {0-1}
     and relative to the image) */
  float thumb_normalized_x;
  float thumb_normalized_y;
  float thumb_image_x;
  float thumb_image_y;
  float fingertip_distance;
  bool fingers_connected; /* are finger tips connected? (distance < THRESH) */
  bool visible; /* is the hand on the screen */
  Handedness handedness;
};


void zoom_and_pan_modality1(HandState hands[2], Gesture& active_gesture,
                            std::string& display_str, cv::Mat& output_frame_mat,
                            float& prev_x, float& prev_y, float& prev_hand_distance,
                            const int sockfd);

void zoom_and_pan_modality2(HandState hands[2], bool& zoom_active, bool& pan_active,
                            std::string& display_str, cv::Mat& output_frame_mat,
                            float& prev_x, float& prev_y, float& zoom_prev_y,
                            const int sockfd, const int hand_count);

absl::Status RunMPPGraph() {
  std::string calculator_graph_config_contents;
  MP_RETURN_IF_ERROR(mediapipe::file::GetContents(
      absl::GetFlag(FLAGS_calculator_graph_config_file),
      &calculator_graph_config_contents));
  LOG(INFO) << "Get calculator graph config contents: "
            << calculator_graph_config_contents;
  mediapipe::CalculatorGraphConfig config =
      mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
          calculator_graph_config_contents);

  LOG(INFO) << "Initialize the calculator graph.";
  mediapipe::CalculatorGraph graph;
  MP_RETURN_IF_ERROR(graph.Initialize(config));

  LOG(INFO) << "Initialize the GPU.";
  ASSIGN_OR_RETURN(auto gpu_resources, mediapipe::GpuResources::Create());
  MP_RETURN_IF_ERROR(graph.SetGpuResources(std::move(gpu_resources)));
  mediapipe::GlCalculatorHelper gpu_helper;
  gpu_helper.InitializeForTest(graph.GetGpuResources().get());

  LOG(INFO) << "Initialize the camera or load the video.";
  cv::VideoCapture capture;
  const bool load_video = !absl::GetFlag(FLAGS_input_video_path).empty();
  if (load_video) {
    capture.open(absl::GetFlag(FLAGS_input_video_path));
  } else {
    capture.open(0);
  }
  RET_CHECK(capture.isOpened());

  cv::VideoWriter writer;
  const bool save_video = !absl::GetFlag(FLAGS_output_video_path).empty();
  if (!save_video) {
    cv::namedWindow(kWindowName, /*flags=WINDOW_AUTOSIZE*/ 1);
#if (CV_MAJOR_VERSION >= 3) && (CV_MINOR_VERSION >= 2)
    capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    capture.set(cv::CAP_PROP_FPS, 30);
#endif
  }

  LOG(INFO) << "Start running the calculator graph.";
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller,
                   graph.AddOutputStreamPoller(kOutputStream));
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller_landmark,
            graph.AddOutputStreamPoller(kLandmarksStream));
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller handedness_poller,
                  graph.AddOutputStreamPoller(kHandedness));
  MP_RETURN_IF_ERROR(graph.StartRun({}));

  /* Create socket and connect to image viewer. */
  char *server_filename = "/tmp/image-viewer-server.sock";
  struct sockaddr_un server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sun_family = AF_UNIX;
  strcpy(server_addr.sun_path, server_filename);
  int sockfd = socket(AF_UNIX, SOCK_DGRAM, 0);
  connect(sockfd, (struct sockaddr *) &server_addr, sizeof(server_addr));


  LOG(INFO) << "Start grabbing and processing frames.";
  bool grab_frames = true;

  /* modality 1 gesture state, saved after each frame */
  Gesture active_gesture = Gesture::None;
  float prev_hand_distance = -1; /* distance between left and right thumb, used for zoom */

  /* modality 2 gesture state */
  float zoom_prev_y = -1; /* keep track of y coordinate to change zoom level. (right hand) */
  bool zoom_active = false, pan_active = false;

  /* used for both modalities */
  float prev_x = -1., prev_y = -1.; /* keep track of fingertip position for panning */

  unsigned modality = 1; /* 1 or 2 */;

  std::string display_str = "IDLE";
  while (grab_frames) {
    // Capture opencv camera or video frame.
    cv::Mat camera_frame_raw;
    capture >> camera_frame_raw;
    if (camera_frame_raw.empty()) {
      if (!load_video) {
        LOG(INFO) << "Ignore empty frames from camera.";
        continue;
      }
      LOG(INFO) << "Empty frame, end of video reached.";
      break;
    }
    cv::Mat camera_frame;
    cv::cvtColor(camera_frame_raw, camera_frame, cv::COLOR_BGR2RGBA);
    if (!load_video) {
      cv::flip(camera_frame, camera_frame, /*flipcode=HORIZONTAL*/ 1);
    }

    // Wrap Mat into an ImageFrame.
    auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
        mediapipe::ImageFormat::SRGBA, camera_frame.cols, camera_frame.rows,
        mediapipe::ImageFrame::kGlDefaultAlignmentBoundary);
    cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
    camera_frame.copyTo(input_frame_mat);

    // Prepare and add graph input packet.
    size_t frame_timestamp_us =
        (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
    MP_RETURN_IF_ERROR(
        gpu_helper.RunInGlContext([&input_frame, &frame_timestamp_us, &graph,
                                   &gpu_helper]() -> absl::Status {
          // Convert ImageFrame to GpuBuffer.
          auto texture = gpu_helper.CreateSourceTexture(*input_frame.get());
          auto gpu_frame = texture.GetFrame<mediapipe::GpuBuffer>();
          glFlush();
          texture.Release();
          // Send GPU image packet into the graph.
          MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(
              kInputStream, mediapipe::Adopt(gpu_frame.release())
                                .At(mediapipe::Timestamp(frame_timestamp_us))));
          return absl::OkStatus();
        }));

    // Get the graph result packet, or stop if that fails.
    mediapipe::Packet packet;
    mediapipe::Packet landmark_packet;
    if (!poller.Next(&packet)) break;
    mediapipe::Packet handedness_packet;
    std::unique_ptr<mediapipe::ImageFrame> output_frame;

    // Convert GpuBuffer to ImageFrame.
    MP_RETURN_IF_ERROR(gpu_helper.RunInGlContext(
        [&packet, &output_frame, &gpu_helper]() -> absl::Status {
          auto& gpu_frame = packet.Get<mediapipe::GpuBuffer>();
          auto texture = gpu_helper.CreateSourceTexture(gpu_frame);
          output_frame = absl::make_unique<mediapipe::ImageFrame>(
              mediapipe::ImageFormatForGpuBufferFormat(gpu_frame.format()),
              gpu_frame.width(), gpu_frame.height(),
              mediapipe::ImageFrame::kGlDefaultAlignmentBoundary);
          gpu_helper.BindFramebuffer(texture);
          const auto info = mediapipe::GlTextureInfoForGpuBufferFormat(
              gpu_frame.format(), 0, gpu_helper.GetGlVersion());
          glReadPixels(0, 0, texture.width(), texture.height(), info.gl_format,
                       info.gl_type, output_frame->MutablePixelData());
          glFlush();
          texture.Release();
          return absl::OkStatus();
        }));

    // Convert back to opencv for display or saving.
    cv::Mat output_frame_mat = mediapipe::formats::MatView(output_frame.get());
    if (output_frame_mat.channels() == 4)
      cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGBA2BGR);
    else
      cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGB2BGR);
    
    const float CONNECTED_THRESH = 22.f/*30.f*/;
    /* Process hand landmark detections */
    if (poller_landmark.QueueSize() > 0) {
      if (!poller_landmark.Next(&landmark_packet)) {
        break;
      }
      if (!handedness_poller.Next(&handedness_packet)) {
        break;
      } 
      auto& output_landmarks =
        landmark_packet.Get<std::vector<mediapipe::NormalizedLandmarkList>>();

      const auto& handedness_list =
        handedness_packet.Get<
          std::vector<mediapipe::ClassificationList,
          std::allocator<mediapipe::ClassificationList>>>(); 
      HandState hands[2] = {};
      unsigned hand_count = 0;


      for (const ::mediapipe::NormalizedLandmarkList& hand_landmarks : output_landmarks) {
        if (hand_count == 2) {
          break; /* Detect no more than two hands. */
        }
        HandState& hand = hands[hand_count];
        hand.visible = true;

        const int THUMB_TIP = 4, INDEX_FINGER_TIP = 8;
        const mediapipe::NormalizedLandmark& thumb_tip = hand_landmarks.landmark(THUMB_TIP);
        const mediapipe::NormalizedLandmark& index_tip = hand_landmarks.landmark(INDEX_FINGER_TIP);
        auto [thumb_x, thumb_y] = normalized_to_pixel_coordinates(
          thumb_tip.x(), thumb_tip.y(), output_frame_mat.cols, output_frame_mat.rows);
        auto [index_x, index_y] = normalized_to_pixel_coordinates(
          index_tip.x(), index_tip.y(), output_frame_mat.cols, output_frame_mat.rows);
        
        /* draw line between fingertips */
        cv::Point p1(thumb_x, thumb_y), p2(index_x, index_y);
        int thickness = 2;
        cv::line(output_frame_mat, p1, p2, cv::Scalar(0, 0, 255), thickness, cv::LINE_8);

        hand.fingertip_distance = distance(thumb_x, thumb_y, index_x, index_y);
        if (hand.fingertip_distance < CONNECTED_THRESH) {
          hand.fingers_connected = true;
        }
        /* save normalized (between 0-1) and image relative thumb tip coordinates */
        hand.thumb_normalized_x = thumb_tip.x();
        hand.thumb_normalized_y = thumb_tip.y();
        hand.thumb_image_x = thumb_x;
        hand.thumb_image_y = thumb_y;

        /* detect handedness */
        auto& h = handedness_list[hand_count];
        if (h.classification_size() > 0) {
          const mediapipe::Classification & c = h.classification(0);
          if (c.label() == "Left") {
            hand.handedness = Left;
          } else {
            hand.handedness = Right;
          }
        } else {
          std::cout << "something is wrong\n";
        }

        hand_count++;
      }

      if (modality == 1) {
        zoom_and_pan_modality1(hands, active_gesture, display_str, output_frame_mat,
                               prev_x, prev_y, prev_hand_distance, sockfd);
      } else {
        zoom_and_pan_modality2(hands, zoom_active, pan_active, display_str,
                              output_frame_mat, prev_x, prev_y, zoom_prev_y,
                              sockfd, hand_count);
      }

    } else {
      active_gesture = Gesture::None;
      display_str = "IDLE";
    }


    cv::Point state_text_pos(30, 50);
    cv::putText(output_frame_mat, display_str, state_text_pos,
        cv::FONT_HERSHEY_SIMPLEX, 1.0,
        cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

    cv::Point modality_text_pos(30, 460);
    std::string modality_str =
      (modality == 1) ? "multi-hand-zoom" : "single-hand-zoom";
    cv::putText(output_frame_mat, modality_str, modality_text_pos,
        cv::FONT_HERSHEY_SIMPLEX, 0.8,
        cv::Scalar(0, 0, 255), 2, cv::LINE_AA); 
    

    if (save_video) {
      if (!writer.isOpened()) {
        LOG(INFO) << "Prepare video writer.";
        writer.open(absl::GetFlag(FLAGS_output_video_path),
                    mediapipe::fourcc('a', 'v', 'c', '1'),  // .mp4
                    capture.get(cv::CAP_PROP_FPS), output_frame_mat.size());
        RET_CHECK(writer.isOpened());
      }
      writer.write(output_frame_mat);
    } else {
      cv::imshow(kWindowName, output_frame_mat);
      // Press any key to exit.
      const int pressed_key = cv::waitKey(1);
      //if (pressed_key >= 0 && pressed_key != 255) grab_frames = false;
      if (pressed_key == 27) {
        grab_frames = false; /* quit on ESC keypress */
      } else if (pressed_key == 's' || pressed_key == 'S') {
        modality = (modality == 1) ? 2 : 1; /* switch active modality */
      }
    }
  }   
  LOG(INFO) << "Shutting down.";
  if (writer.isOpened()) writer.release();
  MP_RETURN_IF_ERROR(graph.CloseInputStream(kInputStream));
  return graph.WaitUntilDone();
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  absl::ParseCommandLine(argc, argv);
  absl::Status run_status = RunMPPGraph();
  if (!run_status.ok()) {
    LOG(ERROR) << "Failed to run the graph: " << run_status.message();
    return EXIT_FAILURE;
  } else {
    LOG(INFO) << "Success!";
  }
  return EXIT_SUCCESS;
}

void zoom_and_pan_modality1(HandState hands[2], Gesture& active_gesture,
                            std::string& display_str, cv::Mat& output_frame_mat,
                            float& prev_x, float& prev_y, float& prev_hand_distance,
                            const int sockfd) {
  
  const float  ZOOM_THRESH = 0.05;//0.07;
  const float PAN_THRESH = 22.f/*30.f*/;
  const float PAN_NOISE_THRESH = 0.0025;
  struct GestureMessage msg = {MessageType::None};
  if (hands[0].fingers_connected && hands[1].fingers_connected) {
    /* Fingers are connected on both hands, zoom-in or zoom-out. */
    if (active_gesture == Gesture::Zoom) {
      /* calc distance ratio, send event */
      float hand_distance = distance(hands[0].thumb_image_x, hands[0].thumb_image_y,
                                     hands[1].thumb_image_x, hands[1].thumb_image_y);
      float distance_ratio = hand_distance / prev_hand_distance;
      if (distance_ratio > 1 + ZOOM_THRESH) {
        display_str = "ZOOM IN";
        msg.type = MessageType::Zoom;
        msg.zoom_in = true;
        prev_hand_distance = hand_distance;
      } else if (distance_ratio < 1 - ZOOM_THRESH) {
        display_str = "ZOOM OUT";
        msg.type = MessageType::Zoom;
        msg.zoom_in = false; /* zoom out */
        prev_hand_distance = hand_distance;
      }
    } else {
      display_str = "ZOOM START";
      active_gesture = Gesture::Zoom;
      /* Calculate initial hand distance for next frame */
      prev_hand_distance = distance(hands[0].thumb_image_x, hands[0].thumb_image_y,
                                    hands[1].thumb_image_x, hands[1].thumb_image_y);
    }  
    /* draw line between left and right thumb */
    cv::Point p1(hands[0].thumb_image_x, hands[0].thumb_image_y);
    cv::Point p2(hands[1].thumb_image_x, hands[1].thumb_image_y);
    int thickness = 2;
    cv::line(output_frame_mat, p1, p2, cv::Scalar(255, 0, 0), thickness, cv::LINE_8);
  } else if (hands[0].fingers_connected ^ hands[1].fingers_connected) {
    /* Fingertips are connected on only one hand - pan the image. */
    HandState& active_hand = (hands[0].fingers_connected) ? hands[0] : hands[1];
    msg.type = MessageType::Move;
    if (active_gesture == Gesture::Pan) {
      display_str = "PAN";
      msg.PanState = PanState::Active;
      /* calculate position difference and prepare msg */
      msg.dx = active_hand.thumb_normalized_x - prev_x;
      std::cout << msg.dx << "\n";
      if (std::abs(msg.dx) < PAN_NOISE_THRESH) {
        msg.dx = 0;
      }
      msg.dy = active_hand.thumb_normalized_y - prev_y;
      std::cout << msg.dy << "\n";
      if (std::abs(msg.dy) < PAN_NOISE_THRESH) {
        msg.dy = 0;
      }
    } else {
      display_str = "START PAN";
      msg.PanState = PanState::Start;
      active_gesture = Gesture::Pan;
    }
    prev_x = active_hand.thumb_normalized_x;
    prev_y = active_hand.thumb_normalized_y;
  } else if (active_gesture == Gesture::Pan && !hands[0].fingers_connected && !hands[1].fingers_connected) {
    /* Image panning is finished. */
    active_gesture = Gesture::None;
    display_str = "END PAN";
    msg.type = MessageType::Move;
    msg.PanState = PanState::Stop;
  } else {
    active_gesture = Gesture::None;
    display_str = "IDLE";
  }

  /* Send message to the image viewer if a gesture event occured */
  if (msg.type == MessageType::Zoom || msg.type == MessageType::Move) {
    std::cout << "state " << msg.PanState << "\n";
    int sent = write(sockfd, &msg, sizeof(msg));
    if (sent > 0) std::cout << sent << "\n";
  }
}

void zoom_and_pan_modality2(HandState hands[2], bool& zoom_active, bool& pan_active,
                            std::string& display_str, cv::Mat& output_frame_mat,
                            float& prev_x, float& prev_y, float& zoom_prev_y,
                            const int sockfd, const int hand_count) {
  struct GestureMessage msg = {MessageType::None};
  HandState& active_hand = hands[0];
  const float ZOOM_THRESH = 0.004;
  const float PAN_NOISE_THRESH = 0.0025;

  for (int i = 0; i < hand_count; i++) {
    msg.type = MessageType::None;
    HandState& hand = hands[i];
    switch (hand.handedness) {
      case Left: {
        /* Pan image. */
        if (hand.fingers_connected) {
          msg.type = MessageType::Move;
          if (pan_active) {
            display_str = "PAN";
            msg.PanState = PanState::Active;
            /* calculate position difference and prepare msg */
            msg.dx = hand.thumb_normalized_x - prev_x;
            std::cout << msg.dx << "\n";
            if (std::abs(msg.dx) < PAN_NOISE_THRESH) {
              msg.dx = 0;
            }
            msg.dy = hand.thumb_normalized_y - prev_y;
            std::cout << msg.dy << "\n";
            if (std::abs(msg.dy) < PAN_NOISE_THRESH) {
              msg.dy = 0;
            }
          } else {
            display_str = "START PAN";
            msg.PanState = PanState::Start;
            pan_active = true;
          }
          prev_x = hand.thumb_normalized_x;
          prev_y = hand.thumb_normalized_y;
        } else {
          if (pan_active) {
            pan_active = false;
          }
        }
        break;
      }
      case Right: {
        /* Zoom image. */
        if (hand.fingers_connected) {
          if (zoom_active) {
            float dy = hand.thumb_normalized_y - zoom_prev_y;
            if (dy > ZOOM_THRESH) {
              display_str = "ZOOM OUT";
              msg.type = MessageType::Zoom;
              msg.zoom_in = false;
            } else if (dy < -ZOOM_THRESH) {
              display_str = "ZOOM IN";
              msg.type = MessageType::Zoom;
              msg.zoom_in = true;
            }
          } else {
            zoom_active = true;
          }
          zoom_prev_y = hand.thumb_normalized_y;
        } else {
          if (zoom_active) {
            zoom_active = false;
          }
        }
        break;
      }
      default: {
        std::cerr << "hand is not classified";
        break;
      }
    }
    /* Send message to the image viewer if a gesture event occured on current hand */
    if (msg.type == MessageType::Zoom || msg.type == MessageType::Move) {
      std::cout << "state " << msg.PanState << "\n";
      int sent = write(sockfd, &msg, sizeof(msg));
      if (sent > 0) std::cout << sent << "\n";
    }
  }
}