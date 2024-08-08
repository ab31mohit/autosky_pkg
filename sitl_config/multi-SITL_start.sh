#!/bin/bash

gnome-terminal -- bash -c "
gnome-terminal --tab --title='Vehicle1' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I0; exec bash'; \
gnome-terminal --tab --title='Vehicle2' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone2 -I1; exec bash'; \
gnome-terminal --tab --title='Vehicle3' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone3 -I2; exec bash'; \
gnome-terminal --tab --title='Vehicle4' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone4 -I3; exec bash'; \
gnome-terminal --tab --title='Vehicle5' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone5 -I4; exec bash'; \
gnome-terminal --tab --title='Vehicle6' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone6 -I5; exec bash'; \
gnome-terminal --tab --title='Vehicle7' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone7 -I6; exec bash'; \
gnome-terminal --tab --title='Vehicle7' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone8 -I7; exec bash';"
