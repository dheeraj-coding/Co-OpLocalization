#!/bin/bash

rosrun coop_localization launch_drone.py _id:=1 _x:=0 _y:=0 _z:=1 _namespace:="/iris1" __ns:="/iris1" __use_sim_time:="true"
sleep 10
rosrun coop_localization launch_drone.py _id:=2 _x:=-0.5 _y:=0.5 _z:=4 _namespace:="/iris2" __ns:="/iris2" __use_sim_time:="true"
sleep 10
rosrun coop_localization launch_drone.py _id:=3 _x:=-0.5 _y:=-0.5 _z:=4 _namespace:="/iris3" __ns:="/iris3" __use_sim_time:="true"
sleep 10
rosrun coop_localization launch_drone.py _id:=4 _x:=0.5 _y:=-0.5 _z:=4 _namespace:="/iris4" __ns:="/iris4" __use_sim_time:="true"
sleep 10
rosrun coop_localization launch_drone.py _id:=5 _x:=0 _y:=0 _z:=7 _namespace:="/iris5" __ns:="/iris5" __use_sim_time:="true"
sleep 10
rosrun coop_localization launch_drone.py _id:=6 _x:=0.5 _y:=0.5 _z:=4 _namespace:="/iris6" __ns:="/iris6" __use_sim_time:="true"
sleep 10

