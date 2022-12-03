
    #!/bin/bash
    konsole --hold --new-tab -e $SHELL -c "sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I0" &konsole --hold --new-tab -e $SHELL -c "sim_vehicle.py -v ArduCopter -f gazebo-drone2 -I1" &konsole --hold --new-tab -e $SHELL -c "sim_vehicle.py -v ArduCopter -f gazebo-drone3 -I2" &konsole --hold --new-tab -e $SHELL -c "sim_vehicle.py -v ArduCopter -f gazebo-drone4 -I3" &konsole --hold --new-tab -e $SHELL -c "sim_vehicle.py -v ArduCopter -f gazebo-drone5 -I4" &
