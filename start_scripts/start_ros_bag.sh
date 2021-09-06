#!/bin/bash

if [ ! -d "/logs" ] 
then
    mkdir logs 
fi
cd logs

ec=0; while (($ec < 2)); do echo "starting..."; sleep 1; ros2 bag record -a -p 100 -b 25000000; (($?==2)) && break; ((ec+=1)); done
