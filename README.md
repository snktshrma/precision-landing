# precision-landing

This repository contains precision landing project for CUAV using Ardupilot.
Click below for simulation tests:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=6HBLpPkoXc4" target="_blank"><img src="http://img.youtube.com/vi/6HBLpPkoXc4 /0.jpg" width="240" height="180" border="10" /></a>


## Code Integration

1. Run the following in terminal on RPi `python3 mavproxy.py --master=<port> --baudrate 921600 --out <ip>:<port>`
2. After setting up code on companion computer and connectiong over Telem port to FCU, run `cuav_landing.py`.
3. Make sure to setup the coordinates of the destination and correct camera parameters and Aruco parameters.
