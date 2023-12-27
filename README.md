# precision-landing

This repository contains precision landing project for CUAV using Ardupilot.
Click below for simulation tests:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=6HBLpPkoXc4" target="_blank"><img src="http://img.youtube.com/vi/6HBLpPkoXc4 /0.jpg" width="240" height="180" border="10" /></a>


## Code Integration



    - Hardware connection: https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html

    - Change the following using Mission Planner
        SERIALx_PROTOCOL = 2 (the default - to enable MAVLink 2 on the serial port)
        SERIALx_BAUD = 921 (921600 baud)

    - Install and flash ubuntu 20.04 image on a memory card and put inside RPi
    
    - SSH into Raspberry Pi and connect camera to RPi and under /boot/config.txt, add start_x=1

    - Run the following commands in RPi terminal to set up MAVProxy:

        sudo apt-get update
        sudo apt-get install python3-pip.
        sudo apt-get install libxml2-dev.
        sudo apt-get install libxslt-dev.
        pip install opencv-python opencv-contrib-python mavproxy pymavlink sockets numpy tf logging python-signal
        sudo apt-get update && sudo apt-get install ffmpeg libsm6 libxext6  -y 

- Run the following in terminal on RPi `python3 mavproxy.py --master=<port> --baudrate 921600 --out <ip>:<port>`
- After setting up code on companion computer and connectiong over Telem port to FCU, run `cuav_landing.py`.
- Make sure to setup the coordinates of the destination and correct camera parameters and Aruco parameters.
