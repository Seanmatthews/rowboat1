# Separated this install script out because it doesn't belong in the base-install script.
# ALso, this takes a long time on the Odroid

git clone https://github.com/Itseez/opencv.git
git clone https://github.com/Itseez/opencv_contrib.git
cd opencv
git checkout tags/3.1.0
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/root/opencv_contrib/modules ..
make -j6
make install
