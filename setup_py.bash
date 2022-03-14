#!/bin/bash

if [[ ! -f "$(pwd)/setup_py.bash" ]]
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
wget "https://download.ifi.uzh.ch/rpg/Flightmare/RPG_Flightmare.zip" | tar Jxf - -C flightmare/flightrender/ --strip 1

echo "export FLIGHTMARE_PATH=$PWD/flightmare" >> ~/.bashrc

# 
echo "Createing an conda environment from the environment.yaml file"
conda env create -f environment.yaml

# 
echo "Source the anaconda environment. If errors, change to the right anaconda path."
source ~/anaconda3/etc/profile.d/conda.sh

# 
echo "Actiavte the environment"
conda activate agileflight

echo "Compiling the agile flight environment and install the environment as python package"
cd $PWD/flightmare/flightlib/build
cmake ..
make -j10
pip install .

echo "Run the first vision demo."
cd ../../../envtest 
python3 -m envpy.run_vision_demo --render 1

echo "Done!"
echo "Have a save flight!"
