#!/bin/bash

if [[ ! -f "$(pwd)/setup.bash" ]]
then
  echo "please launch from the agile_flight folder!"
  exit
fi

echo "Making sure submodules are initialized and up-to-date"
git submodule update --init --recursive

echo "Using apt to install dependencies..."
echo "Will ask for sudo permissions:"
sudo apt update
sudo apt install -y --no-install-recommends build-essential cmake libzmqpp-dev libopencv-dev 

echo "Ignoring unused Flightmare folders!"
touch flightmare/flightros/CATKIN_IGNORE

echo "Downloading Flightmare Unity standalone..."
curl --show-error --progress-bar --location "https://github.com/uzh-rpg/flightmare/releases/download/0.0.5/RPG_Flightmare.tar.xz" | tar Jxf - -C flightmare/flightrender/ --strip 1

echo "export FLIGHTMARE_PATH=$PWD/flightmare" >> ~/.bashrc

# 
echo "Createing an conda environment from the environment.yaml file"
conda env create -f environment.yaml
conda activate agileflight

echo "Compiling the agile flight environment and install the environment as python package"
cd $PWD/flightmare/flightlib/build
cmake ..
make -j10
pip install .

echo "Done!"
echo "Have a save flight!"
