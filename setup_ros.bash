#!/bin/bash

if [[ ! -f "$(pwd)/setup_ros.bash" ]]
then
  echo "please launch from the racing folder!"
  exit
fi

echo "Making sure submodules are initialized and up-to-date"
git submodule update --init --recursive


echo "Using apt to install dependencies..."
echo "Will ask for sudo permissions:"
sudo apt update
sudo apt install -y --no-install-recommends build-essential cmake libzmqpp-dev libopencv-dev unzip
sudo pip install gdown

echo "Ignoring unused Flightmare folders!"
touch flightmare/flightros/CATKIN_IGNORE

echo "Downloading Flightmare Unity standalone..."
gdown --id 1ROT6EVmsdDHXM6hnMxM8cHQLStDlak_E
unzip FlightmareSimple.zip -d flightmare/flightrender
rm FlightmareSimple.zip


curl --show-error --progress-bar --location "https://github.com/uzh-rpg/flightmare/releases/download/0.0.5/RPG_Flightmare.tar.xz" | tar Jxf - -C flightmare/flightrender/ --strip 1

echo "Setting the flightmare environment variable. Please add 'export FLIGHTMARE_PATH=$PWD/flightmare' to your .bashrc!"
export FLIGHTMARE_PATH=$PWD/flightmare

echo "Done!"
echo "Have a save flight!"
