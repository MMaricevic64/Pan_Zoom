# Copyright 2019 The MediaPipe Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

licenses(["notice"])

package(default_visibility = [
    "//visibility:public",
])

# Linux only.
# Must have a GPU with EGL support:
# ex: sudo apt-get install mesa-common-dev libegl1-mesa-dev libgles2-mesa-dev
# (or similar nvidia/amd equivalent)

cc_library(
    name = "control_image_viewer_gpu",
    srcs = ["linux/control_image_viewer_gpu.cc"],
    deps = [
        "//mediapipe/calculators/util:landmarks_to_render_data_calculator",
        "//mediapipe/framework/formats:classification_cc_proto",
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework/formats:landmark_cc_proto",
        "//mediapipe/framework/port:file_helpers",
        "//mediapipe/framework/port:opencv_highgui",
        "//mediapipe/framework/port:opencv_imgproc",
        "//mediapipe/framework/port:opencv_video",
        "//mediapipe/framework/port:parse_text_proto",
        "//mediapipe/framework/port:status",
        "//mediapipe/gpu:gl_calculator_helper",
        "//mediapipe/gpu:gpu_buffer",
        "//mediapipe/gpu:gpu_shared_data_internal",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
    ],
)

cc_binary(
    name = "image_viewer_hand_tracking_gpu",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop/pan_zoom:control_image_viewer_gpu",
        "//mediapipe/graphs/hand_tracking:mobile_calculators",
    ],
)

# Windows
cc_library(
    name = "control_image_viewer_cpu",
    srcs = ["windows/control_image_viewer_cpu.cc"],
    deps = [
        "//mediapipe/calculators/util:landmarks_to_render_data_calculator",
        "//mediapipe/framework/formats:classification_cc_proto",
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework/formats:landmark_cc_proto",
        "//mediapipe/framework/port:file_helpers",
        "//mediapipe/framework/port:opencv_highgui",
        "//mediapipe/framework/port:opencv_imgproc",
        "//mediapipe/framework/port:opencv_video",
        "//mediapipe/framework/port:parse_text_proto",
        "//mediapipe/framework/port:status",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
    ],
    hdrs = glob(["windows/afunix.h"]),
)

cc_binary(
    name = "image_viewer_hand_tracking_cpu",
    data = [
        "//mediapipe/modules/hand_landmark:hand_landmark_full.tflite",
        "//mediapipe/modules/palm_detection:palm_detection_full.tflite",
    ],
    deps = [
        "//mediapipe/examples/desktop/pan_zoom:control_image_viewer_cpu",
        "//mediapipe/graphs/hand_tracking:desktop_tflite_calculators",
    ],
)
