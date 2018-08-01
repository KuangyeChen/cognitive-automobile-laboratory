#!/bin/bash

SOURCE_DIR=$1

export PYENV_ROOT="${SOURCE_DIR}/../../build/kitaf/kal3python"

export PATH="$PYENV_ROOT/bin:$PATH"
if command -v pyenv 1>/dev/null 2>&1; then
  eval "$(pyenv init -)"
fi

export PATH="/usr/local/cuda/bin:/usr/local/cuda-8.0/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:/usr/local/cuda-8.0/lib64:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="$SOURCE_DIR/cuda/lib64:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/cuda/extras/CUPTI/lib64:/usr/local/cuda-8.0/extras/CUPTI/lib64"

pyenv shell 2.7.15
roslaunch kitaf_detector_ros_tool kitaf_detector_node.launch