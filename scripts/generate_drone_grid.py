import argparse

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

def main():
    global template
    parser = argparse.ArgumentParser(description="rows = N \n cols = N")
    parser.add_argument('--rows', type=int, default=2)
    parser.add_argument('--cols', type=int, default=2)
    parser.add_argument('--path', type=str, default='runway.launch')

    args = parser.parse_args()

    modelTemplate = '''
    <model name="iris{id}">
      <include>
        <uri>model://drone_with_camera_qr</uri>
      </include>
      <pose> {x} {y} 0 0 0 0</pose>
    </model>
    '''

    result = ""

    id = 0
    for i in range(args.rows):
        for j in range(args.cols):
            result += modelTemplate.format(id=id, x=i, y=j)
            id += 1
    
    output = template.format(drone_models=result)

    with open(args.path, 'w+') as f:
        f.write(output)
        

    




if __name__ == "__main__":
    main()