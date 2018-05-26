sudo apt-get update
sudo apt-get install build-essential cmake
sudo apt-get install libopenblas-dev liblapack-dev libatlas-base-dev
sudo apt-get install libx11-dev libgtk-3-dev
pip install numpy
pip install scipy
pip install scikit-image
git clone https://github.com/davisking/dlib.git
cd dlib
sudo python setup.py install --yes USE_NEON_INSTRUCTIONS
