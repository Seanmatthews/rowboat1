# Separated this install script out because it doesn't belong in the base-install script.
# ALso, this takes a long time on the Odroid

# OpenCV
${PKG} libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
${PKG} python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
cd
git clone https://github.com/Itseez/opencv.git
git clone https://github.com/Itseez/opencv_contrib.git
cd opencv_contrib
git checkout 3.1.0
cd ../opencv
git checkout 3.1.0
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..
make -j6
make install
cd
rm -rf opencv
rm -rf opencv_contrib

