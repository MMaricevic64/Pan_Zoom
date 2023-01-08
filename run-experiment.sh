#!/usr/bin/bash

run_image_viewer_hand_tracking() {
  if [ $1 == "linux" ]; then
    cd mediapipe/
    GLOG_logtostderr=1
    bazel-bin/mediapipe/examples/desktop/pan_zoom/image_viewer_hand_tracking_gpu --calculator_graph_config_file=mediapipe/graphs/hand_tracking/hand_tracking_desktop_live_gpu.pbtxt &
    cd -
  elif [ $1 == "windows" ]; then
    #Windows CPU run
    cd mediapipe
    GLOG_logtostderr=1
    bazel-bin/mediapipe/examples/desktop/pan_zoom/image_viewer_hand_tracking_cpu --calculator_graph_config_file=mediapipe/graphs/hand_tracking/hand_tracking_desktop_live.pbtxt
    cd ..
  else
    echo -e "Provide an valid argument:\nWindows run: windows\nLinux run: linux\n"
    exit 1
  fi
}

read -p "Enter username (npr. iivic za Ivo Ivić): " USERNAME

LOGFILE=results/concatenated-log.csv
USER_LOGFILE="results/${USERNAME}.csv"
echo "username, test-sequence, test-image, modality, time-milliseconds" >> $USER_LOGFILE

if [ $# -eq 2 ]; then
  IMG_FILELIST=$2
else
  echo "filelist missing"
  exit 1
fi

# run test sequence
if [ $1 == "linux" ]; then
  ./image-viewer/build/example/viewer-example "$IMG_FILELIST" "$USERNAME" | tee -a $USER_LOGFILE $LOGFILE &
  VIEWER_PID=$!
  # wait for image viewer initialization to complete before attempting to connect from the hand tracking program
  # (increase sleep amount if necessary)
  sleep 1
elif [ $1 == "windows" ]; then
  ./image-viewer/build/example/Release/viewer-example "$IMG_FILELIST" "$USERNAME" | tee -a $USER_LOGFILE $LOGFILE &
  VIEWER_PID=$!
  # wait for image viewer initialization to complete before attempting to connect from the hand tracking program
  # (increase sleep amount if necessary)
  sleep 1
fi

run_image_viewer_hand_tracking $1 > /dev/null 2>&1
HAND_TRACKING_PID=$!
wait $VIEWER_PID
kill $HAND_TRACKING_PID
