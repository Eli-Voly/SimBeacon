#!/bin/sh

    pkill -9 mavlink
    pkill -9 QGroundControl

    export PX4_HOME_LAT=37.9024128
    export PX4_HOME_LON=-121.4983975
    export PX4_HOME_ALT=.1
    export PX4_SIM_SPEED_FACTOR=6

    mavlink-routerd 127.0.0.1:14550 &
    sleep 1
    cd ~ && ./Documents/QGroundControl.PLD.AppImage &
    sleep 1
    cd ~/Simbeacon && python3 SimBeacon.py &
    sleep 1
    cd ~/Autopilot && HEADLESS=1 make px4_sitl gazebo_standard_vtol
