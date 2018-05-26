sudo apt-get -y purge wolfram-engine
sudo apt-get -y purge libreoffice*
sudo apt-get clean
sudo apt-get autoremove
sudo apt-get -y update && sudo apt-get -y upgrade
sudo apt-get -y install build-essential cmake cmake-curses-gui  pkg-config
sudo apt-get -y install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev
sudo apt-get -y install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libeigen3-dev
sudo apt-get -y install libv4l-dev v4l-utils
sudo modprobe bcm2835-v4l2
sudo apt-get -y install libxvidcore-dev libx264-dev
sudo apt-get -y install libgtk2.0-dev libgtk-3-dev
sudo apt-get -y install libcanberra-gtk*
sudo apt-get -y install libatlas-base-dev gfortran
sudo rm /usr/bin/python
sudo ln -s /usr/bin/python3.5  /usr/bin/python
cd ~
wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.3.0.zip
unzip opencv.zip
wget -O opencv_contrib.zip https://github.com/Itseez/opencv_contrib/archive/3.3.0.zip
unzip opencv_contrib.zip
wget https://bootstrap.pypa.io/get-pip.py
sudo python get-pip.py
sudo pip3 install numpy
cd ~/opencv-3.3.0/
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-3.3.0/modules \
    -D ENABLE_NEON=ON \
    -D ENABLE_VFPV3=ON \
    -D BUILD_TESTS=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D BUILD_EXAMPLES=OFF ..

echo "CONF_SWAPSIZE=1024" | sudo tee /etc/dphys-swapfile
sudo /etc/init.d/dphys-swapfile restart
make -j4
sudo make install
echo "CONF_SWAPSIZE=100" | sudo tee /etc/dphys-swapfile
sudo /etc/init.d/dphys-swapfile restart

