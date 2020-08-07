#!/bin/bash

#HOME='/home/pi/repo/mjpg-streamer/mjpg-streamer-experimental/'
#SCRIPT=$HOME/mjpg_streamer
#cd $HOME
#export LD_LIBRARY_PATH=.

/usr/local/bin/mjpg_streamer  -o "output_http.so -w ./www" -i "input_raspicam.so -x 320 -y 260 -fps 10"
