#!/bin/bash

function install_package {
    local package_name=$1
    echo -n -e "\e[1;34mInstalling $package_name...\e[0m"
    pip install $package_name > /dev/null 2>&1 &
    
    local pid=$!
    local delay=0.5
    local spin='-\|/'
    local i=0
    
    while ps -p $pid > /dev/null; do
        local i=$(( (i+1) %4 ))
        printf "\r%s" "${spin:$i:1}"
        sleep $delay
    done
    
    wait $pid
    if [ $? -eq 0 ]; then
        echo -e "\r\e[1;32mInstalling $package_name... Done.\e[0m"
    else
        echo -e "\r\e[1;31mError installing $package_name.\e[0m"
        exit 1
    fi
}

echo -e "\e[1;34mInstalling OpenCV...\e[0m"
install_package "opencv-python opencv-contrib-python"

echo -e "\e[1;34mInstalling pymavlink...\e[0m"
install_package "pymavlink"

echo -e "\e[1;34mInstalling sockets, numpy, tf, logging, python-signal...\e[0m"
install_package "sockets"
install_package "numpy"
install_package "tf"
install_package "logging"
install_package "python-signal"

echo -e "\e[1;34mUpdating and installing dependencies...\e[0m"
sudo apt-get update
sudo apt-get install ffmpeg libsm6 libxext6 -y
if [ $? -eq 0 ]; then
    echo -e "\e[1;32mDependencies installed successfully.\e[0m"
else
    echo -e "\e[1;31mError installing dependencies.\e[0m"
    exit 1
fi

read -p "Do you want to clone the precision landing repository? (y/n): " clone_choice
if [ "$clone_choice" == "y" ]; then
    echo -e "\e[1;34mCloning precision landing repository...\e[0m"
    git clone https://github.com/snktshrma/precision-landing
    if [ $? -eq 0 ]; then
        echo -e "\e[1;32mRepository cloned successfully.\e[0m"
    else
        echo -e "\e[1;31mError cloning repository.\e[0m"
        exit 1
    fi
else
    echo -e "\e[1;33mSkipping repository cloning as per user choice.\e[0m"
fi

echo -e "\e[1;32mSetup completed successfully.\e[0m"
