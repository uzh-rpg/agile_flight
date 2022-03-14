#!/bin/bash

if [[ ! -f "$(pwd)/setup_ros.bash" ]]
then
  echo "please launch from the racing folder!"
  exit
fi

project_path=$PWD
echo $project_path

echo "Making sure submodules are initialized and up-to-date"
git submodule update --init --recursive

echo "Using apt to install dependencies..."
echo "Will ask for sudo permissions:"
sudo apt update
sudo apt install -y --no-install-recommends build-essential cmake libzmqpp-dev libopencv-dev unzip python3-catkin-tools
sudo pip install uniplot

echo "Ignoring unused Flightmare folders!"
touch flightmare/flightros/CATKIN_IGNORE

echo "Downloading Flightmare Unity standalone..."
wget "https://download.ifi.uzh.ch/rpg/Flightmare/RPG_Flightmare.zip" --directory-prefix=$project_path/flightmare/flightrender 

echo "Unziping Flightmare Unity Standalone"
unzip -o $project_path/flightmare/flightrender/RPG_Flightmare.zip -d $project_path/flightmare/flightrender 

echo "Removing Flightmare Unity Standalone zip file"
rm $project_path/flightmare/flightrender/RPG_Flightmare.zip

echo "Setting the flightmare environment variable. Please add 'export FLIGHTMARE_PATH=$PWD/flightmare' to your .bashrc!"
export FLIGHTMARE_PATH=$project_path/flightmare

echo "Done!"
echo "Have a save flight!"
