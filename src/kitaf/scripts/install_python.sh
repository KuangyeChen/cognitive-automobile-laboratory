#!/bin/bash

BUILD_DIR=$1
SOURCE_DIR=$2

if [ -d "${BUILD_DIR}/kal3python" ]; then
  exit 0
fi

export PYENV_ROOT="${BUILD_DIR}/kal3python"
git clone https://github.com/pyenv/pyenv.git ${PYENV_ROOT}

export PATH="$PYENV_ROOT/bin:$PATH"
if command -v pyenv 1>/dev/null 2>&1; then
  eval "$(pyenv init -)"
fi

pyenv install 2.7.15
pyenv shell 2.7.15
pip install --no-cache-dir --disable-pip-version-check numpy scipy numpy-quaternion numba pyyaml rospkg catkin_pkg
pip install --no-cache-dir --disable-pip-version-check $2/res/tensorflow-1.6.0rc0-cp27-cp27mu-linux_x86_64.whl