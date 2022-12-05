import argparse
import os


template = '''
<group>
		<node pkg="coop_localization" type="square" name="square{id}" output="screen" ns="/iris{id}">
			<param name="namespace" value="/iris{id}"/>
			<param name="use_sim_time"  value="true" />
            <param name="id" value="{id}" />
            <param name="height" value="{height}" />
		</node>
</group>
'''

mavrosTemplate = '''
<node pkg="mavros" type="mavros_node" name="mavros" required="false" clear_params="true" output="screen" respawn="true" ns="/iris{id}">
		<param name="fcu_url" value="udp://127.0.0.1:{p1}@{p2}" />
		<param name="gcs_url" value="" />
		<param name="target_system_id" value="{id}" />
		<param name="target_component_id" value="1" />
		<param name="fcu_protocol" value="v2.0" />

		<!-- load blacklist, config -->
		<rosparam command="load" file="$(find mavros)/launch/apm_pluginlists.yaml" />
		<rosparam command="load" file="$(find mavros)/launch/apm_config.yaml" />
</node>
<group ns="/iris{id}">
    <node pkg="coop_localization" type="positionService.py" name="position" output="screen" ns="/iris{id}">
        <param name="use_sim_time" value="true" />
        <param name="id" value="{id}" />
    </node>
    <node pkg="coop_localization" type="ekf_fuser.py" name="ekf" output="screen" ns="/iris{id}">
        <param name="id" value="{id}" />
    </node>
</group>
'''

def main():
    global template
    parser = argparse.ArgumentParser(description="output-path")
    parser.add_argument("--outputpath", type=str, default="")
    parser.add_argument("--numdrones", type=int, default=1)

    args = parser.parse_args()

    droneLaunchFile = os.path.join(args.outputpath, "multi_drone.launch")
    result = ""

    height = 2
    for i in range(args.numdrones):
        result += template.format(id = str(i+1), height=str(height))
        height += 2
    
    try:
        with open(droneLaunchFile, "w+") as f:
            temp = "<launch> {template} </launch>".format(template=result)
            f.write(temp)
        print("Successfully wrote drone launch files")
    except e:
        print("Failed to write drone launch file")

    mavrosLaunchFile = os.path.join(args.outputpath, "multi_mavros.launch")
    result = ""
    udp_port = 14551

    for i in range(args.numdrones):
        result += mavrosTemplate.format(id = str(i+1), p1=udp_port, p2=udp_port+4)
        udp_port += 10
    try:
        with open(mavrosLaunchFile, "w+") as f:
            temp = "<launch> {template} </launch>".format(template=result)
            f.write(temp)
        print("Successsfully wrote mavros launch file")
    except e:
        print("Failed to write mavros launch file")
    
    arducopterScript = '''
    #!/bin/bash
    '''
    scriptTemplate = 'konsole --hold --new-tab -e $SHELL -c "sim_vehicle.py -v ArduCopter -f gazebo-drone{droneNum} -I{id}" &'
    result = ""
    port = 8100
    for i in range(args.numdrones):
        result = scriptTemplate.format(id = str(i), droneNum=str(i+1))
        port += 100
        arducopterScript += result
    try: 
        with open("ardulaunch.sh", "w+") as f:
            f.write(arducopterScript)
        print("Successfully wrote arducopter script")
    except e:
        print("Failed to write arducopter script")
    
    
    
if __name__ == "__main__":
    main()
    
    


    

    