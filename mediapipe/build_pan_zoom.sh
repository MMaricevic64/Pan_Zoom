#!/usr/bin/bash
if [ $1 == "linux" ]; then
  #Linux GPU build
  bazel build -c opt --copt -DMESA_EGL_NO_X11_HEADERS --copt -DEGL_NO_X11 mediapipe/examples/desktop/pan_zoom:image_viewer_hand_tracking_gpu
elif [ $1 == "windows" ] && [ ! -z "$2" ]; then
  #Windows CPU build
  bazel build -c opt --define MEDIAPIPE_DISABLE_GPU=1 --action_env PYTHON_BIN_PATH=$2 mediapipe/examples/desktop/pan_zoom:image_viewer_hand_tracking_cpu
else
  echo -e "Provide an valid arguments:\nWindows build (2 arguments): windows C://path//to//python.exe (path to Python .exe file using //)\nLinux build (1 argument): linux\n"
fi
