#!/usr/bin/bash
if [ $1 == "linux" ]; then
  #Linux GPU run
  GLOG_logtostderr=1 bazel-bin/mediapipe/examples/desktop/pan_zoom/image_viewer_hand_tracking_gpu \
    --calculator_graph_config_file=mediapipe/graphs/hand_tracking/hand_tracking_desktop_live_gpu.pbtxt
elif [ $1 == "windows" ]; then
  #Windows CPU run
  GLOG_logtostderr=1
  bazel-bin/mediapipe/examples/desktop/pan_zoom/image_viewer_hand_tracking_cpu --calculator_graph_config_file=mediapipe/graphs/hand_tracking/hand_tracking_desktop_live.pbtxt
else
  echo -e "Provide an valid argument:\nWindows run: windows\nLinux run: linux\n"
fi
