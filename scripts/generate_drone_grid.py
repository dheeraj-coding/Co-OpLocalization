import argparse
import os
import qrcode
import shutil

template = '''
<?xml version="1.0"?> 
<sdf version="1.5">
  <world name="default">
    <physics type="ode">
      <ode>
        <solver>
          <type>quick</type>
          <iters>100</iters>
          <sor>1.0</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.9</erp>
          <contact_max_correcting_vel>0.1</contact_max_correcting_vel>
          <contact_surface_layer>0.0</contact_surface_layer>
        </constraints>
      </ode>
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.0010</max_step_size>
    </physics>

    <scene>
      <sky>
        <clouds>
          <speed>12</speed>
        </clouds>
      </sky>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.25 0.25 0.25 1</background>
    </scene>


    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>5000 5000</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="runway">
          <pose>000 0 0.005 0 0 0</pose>
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1829 45</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Runway</name>
            </script>
          </material>
        </visual>

        <visual name="grass">
          <pose>0 0 -0.1 0 0 0</pose>
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>5000 5000</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grass</name>
            </script>
          </material>
        </visual>

      </link>
    </model>

    <include>
      <uri>model://sun</uri>
    </include>

    {drone_models}

  </world>
</sdf>
'''

sdfTemplate = '''
<?xml version='1.0'?>
<sdf version="1.6" >
  <model name="drone_with_camera">
    <include>
      <uri>model://iris_base_modified</uri>
    </include>

    <link name="cam_link">
      <pose>0 0 -0.02 0 0 0 </pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <!-- <collision name="collision">
          <geometry>
            <cylinder>
              <radius>.005</radius>
              <length>.018</length>
            </cylinder>
          </geometry>
        </collision> -->
      <visual name='cam_link'>
    	<pose>0 0 0.15 0 1.5707 0</pose>
    	<geometry>
    		<cylinder>
    			<radius>.005</radius>
    			<length>.018</length>
    		</cylinder>
    	</geometry>
      </visual>
      <sensor name="cam" type="camera" >
      <pose>0 0 0.15 0 0 0</pose>
      <camera>
        <horizontal_fov>1.2</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>1000</far>
        </clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <always_on>true</always_on>
        <update_rate>30</update_rate>
        <cameraName>webcam</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera_link</frameName>
      </plugin>
      </sensor>
    </link>
    <joint name="camera_mount" type="fixed">
      <child>cam_link</child>
      <parent>iris::base_link</parent>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <upper>0</upper>
          <lower>0</lower>
        </limit>
      </axis>
    </joint>

    <joint name="box_mount" type="fixed">
      <child>box_link</child>
      <parent>iris::base_link</parent>
      <axis>
        <xyz>0 0 5</xyz>
        <limit>
          <upper>0</upper>
          <lower>0</lower>
        </limit>
      </axis>
    </joint>
    <link name="box_link">
      <pose> 0 0 0.35 0 0 0 </pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <visual name="bx_link">
      <pose>0 0 0 0 0 0</pose>
      <geometry>
        <mesh>
          <uri>model://{droneModel}/meshes/qrbox.dae</uri>
        </mesh>
      </geometry>
      <!-- <material>
        <script>
        <uri>file://media/materials/script/gazebo.material</uri>
        <name>Gazebo/Grey</name>
        </script>
      </material> -->
        
      </visual>
    </link>

 

    <!-- plugins -->
    <plugin name="rotor_0_blade_1" filename="libLiftDragPlugin.so">
      <a0>0.3</a0>
      <alpha_stall>1.4</alpha_stall>
      <cla>4.2500</cla>
      <cda>0.10</cda>
      <cma>0.00</cma>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0.0</cda_stall>
      <cma_stall>0.0</cma_stall>
      <area>0.002</area>
      <air_density>1.2041</air_density>
      <cp>0.084 0 0</cp>
      <forward>0 1 0</forward>
      <upward>0 0 1</upward>
      <link_name>iris::rotor_0</link_name>
    </plugin>
    <plugin name="rotor_0_blade_2" filename="libLiftDragPlugin.so">
      <a0>0.3</a0>
      <alpha_stall>1.4</alpha_stall>
      <cla>4.2500</cla>
      <cda>0.10</cda>
      <cma>0.00</cma>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0.0</cda_stall>
      <cma_stall>0.0</cma_stall>
      <area>0.002</area>
      <air_density>1.2041</air_density>
      <cp>-0.084 0 0</cp>
      <forward>0 -1 0</forward>
      <upward>0 0 1</upward>
      <link_name>iris::rotor_0</link_name>
    </plugin>

    <plugin name="rotor_1_blade_1" filename="libLiftDragPlugin.so">
      <a0>0.3</a0>
      <alpha_stall>1.4</alpha_stall>
      <cla>4.2500</cla>
      <cda>0.10</cda>
      <cma>0.00</cma>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0.0</cda_stall>
      <cma_stall>0.0</cma_stall>
      <area>0.002</area>
      <air_density>1.2041</air_density>
      <cp>0.084 0 0</cp>
      <forward>0 1 0</forward>
      <upward>0 0 1</upward>
      <link_name>iris::rotor_1</link_name>
    </plugin>
    <plugin name="rotor_1_blade_2" filename="libLiftDragPlugin.so">
      <a0>0.3</a0>
      <alpha_stall>1.4</alpha_stall>
      <cla>4.2500</cla>
      <cda>0.10</cda>
      <cma>0.00</cma>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0.0</cda_stall>
      <cma_stall>0.0</cma_stall>
      <area>0.002</area>
      <air_density>1.2041</air_density>
      <cp>-0.084 0 0</cp>
      <forward>0 -1 0</forward>
      <upward>0 0 1</upward>
      <link_name>iris::rotor_1</link_name>
    </plugin>

    <plugin name="rotor_2_blade_1" filename="libLiftDragPlugin.so">
      <a0>0.3</a0>
      <alpha_stall>1.4</alpha_stall>
      <cla>4.2500</cla>
      <cda>0.10</cda>
      <cma>0.00</cma>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0.0</cda_stall>
      <cma_stall>0.0</cma_stall>
      <area>0.002</area>
      <air_density>1.2041</air_density>
      <cp>0.084 0 0</cp>
      <forward>0 -1 0</forward>
      <upward>0 0 1</upward>
      <link_name>iris::rotor_2</link_name>
    </plugin>
    <plugin name="rotor_2_blade_2" filename="libLiftDragPlugin.so">
      <a0>0.3</a0>
      <alpha_stall>1.4</alpha_stall>
      <cla>4.2500</cla>
      <cda>0.10</cda>
      <cma>0.00</cma>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0.0</cda_stall>
      <cma_stall>0.0</cma_stall>
      <area>0.002</area>
      <air_density>1.2041</air_density>
      <cp>-0.084 0 0</cp>
      <forward>0 1 0</forward>
      <upward>0 0 1</upward>
      <link_name>iris::rotor_2</link_name>
    </plugin>

    <plugin name="rotor_3_blade_1" filename="libLiftDragPlugin.so">
      <a0>0.3</a0>
      <alpha_stall>1.4</alpha_stall>
      <cla>4.2500</cla>
      <cda>0.10</cda>
      <cma>0.00</cma>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0.0</cda_stall>
      <cma_stall>0.0</cma_stall>
      <area>0.002</area>
      <air_density>1.2041</air_density>
      <cp>0.084 0 0</cp>
      <forward>0 -1 0</forward>
      <upward>0 0 1</upward>
      <link_name>iris::rotor_3</link_name>
    </plugin>
    <plugin name="rotor_3_blade_2" filename="libLiftDragPlugin.so">
      <a0>0.3</a0>
      <alpha_stall>1.4</alpha_stall>
      <cla>4.2500</cla>
      <cda>0.10</cda>
      <cma>0.00</cma>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0.0</cda_stall>
      <cma_stall>0.0</cma_stall>
      <area>0.002</area>
      <air_density>1.2041</air_density>
      <cp>-0.084 0 0</cp>
      <forward>0 1 0</forward>
      <upward>0 0 1</upward>
      <link_name>iris::rotor_3</link_name>
    </plugin>
    <plugin name="arducopter_plugin" filename="libArduPilotPlugin.so">
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>9002</fdm_port_in>
      <fdm_port_out>9003</fdm_port_out>
      <!--
          Require by APM :
          Only change model and gazebo from XYZ to XY-Z coordinates
      -->
      <modelXYZToAirplaneXForwardZDown>0 0 0 3.141593 0 0</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED>0 0 0 3.141593 0 0</gazeboXYZToNED>
      <imuName>iris::iris/imu_link::imu_sensor</imuName>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <control channel="0">
      <!--
          incoming control command [0, 1]
          so offset it by 0 to get [0, 1]
          and divide max target by 1.
          offset = 0
          multiplier = 838 max rpm / 1 = 838
        -->
        <type>VELOCITY</type>
        <offset>0</offset>
        <p_gain>0.20</p_gain>
        <i_gain>0</i_gain>
        <d_gain>0</d_gain>
        <i_max>0</i_max>
        <i_min>0</i_min>
        <cmd_max>2.5</cmd_max>
        <cmd_min>-2.5</cmd_min>
        <jointName>iris::rotor_0_joint</jointName>
        <multiplier>838</multiplier>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>
      <control channel="1">
        <type>VELOCITY</type>
        <offset>0</offset>
        <p_gain>0.20</p_gain>
        <i_gain>0</i_gain>
        <d_gain>0</d_gain>
        <i_max>0</i_max>
        <i_min>0</i_min>
        <cmd_max>2.5</cmd_max>
        <cmd_min>-2.5</cmd_min>
        <jointName>iris::rotor_1_joint</jointName>
        <multiplier>838</multiplier>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>
      <control channel="2">
        <type>VELOCITY</type>
        <offset>0</offset>
        <p_gain>0.20</p_gain>
        <i_gain>0</i_gain>
        <d_gain>0</d_gain>
        <i_max>0</i_max>
        <i_min>0</i_min>
        <cmd_max>2.5</cmd_max>
        <cmd_min>-2.5</cmd_min>
        <jointName>iris::rotor_2_joint</jointName>
        <multiplier>-838</multiplier>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>
      <control channel="3">
        <type>VELOCITY</type>
        <offset>0</offset>
        <p_gain>0.20</p_gain>
        <i_gain>0</i_gain>
        <d_gain>0</d_gain>
        <i_max>0</i_max>
        <i_min>0</i_min>
        <cmd_max>2.5</cmd_max>
        <cmd_min>-2.5</cmd_min>
        <jointName>iris::rotor_3_joint</jointName>
        <multiplier>-838</multiplier>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>
    </plugin>

  </model>
</sdf>
'''

def main():
    global template
    parser = argparse.ArgumentParser(description="rows = N \n cols = N")
    parser.add_argument('--rows', type=int, default=2)
    parser.add_argument('--cols', type=int, default=2)
    parser.add_argument('--path', type=str, default='runway.world')
    parser.add_argument('--modelpath', type=str, default='')

    args = parser.parse_args()

    dronemodelPath = os.path.join(args.modelpath, 'drone_with_camera_qr')

    modelTemplate = '''
    <model name="iris{id}">
      <include>
        <uri>model://{droneModel}</uri>
      </include>
      <pose> {x} {y} 0 0 0 0</pose>
    </model>
    '''

    result = ""

    id = 0
    for i in range(args.rows):
        for j in range(args.cols):
          if args.modelpath == '':
            result += modelTemplate.format(id=id, x=i, y=j)
            id += 1
          else:
            id += 1
            try:
              shutil.copytree(dronemodelPath, dronemodelPath+str(id))
              img = qrcode.make(id)
              img.save(os.path.join(dronemodelPath+str(id), 'meshes/testqr.png'))
              sdfPath = os.path.join(dronemodelPath+str(id), 'model.sdf')
              with open(sdfPath, 'w+') as f:
                f.write(sdfTemplate.format(droneModel = 'drone_with_camera_qr'+str(id)))
            except OSError as er:
              print(er)
              print("Error copying directory! Ensure model path is correct")
            result += modelTemplate.format(id=id, x=i, y=j, droneModel='drone_with_camera_qr'+str(id))

    
    output = template.format(drone_models=result)

    with open(args.path, 'w+') as f:
        f.write(output)
        

    




if __name__ == "__main__":
    main()