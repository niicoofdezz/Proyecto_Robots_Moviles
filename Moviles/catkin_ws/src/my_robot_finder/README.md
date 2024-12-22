<sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <model name='grey_wall'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>3.74092 8.10152 0 0 -0 0</pose>
    </model>
    <model name='grey_wall_clone'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>6.78028 4.25294 0 0 -0 0</pose>
    </model>
    <model name='grey_wall_clone_clone'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>0.031391 4.38886 0 0 0 -1.55997</pose>
    </model>
    <model name='grey_wall_clone_clone_clone'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>-8.19924 1.15864 0 0 0 -1.55997</pose>
    </model>
    <model name='grey_wall_clone_0'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>3.77466 0.772472 0 0 -0 0</pose>
    </model>
    <model name='grey_wall_clone_0_clone'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>-8.48954 0.766627 0 0 -0 0</pose>
    </model>
    <model name='grey_wall_clone_0_clone_clone'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>-6.41833 -6.6617 0 0 -0 0</pose>
    </model>
    <model name='grey_wall_clone_clone_clone_clone'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>-1.3256 -4.543 0 0 0 -1.55997</pose>
    </model>
    <model name='grey_wall_0'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>-2.3204 -6.67079 0 0 -0 0</pose>
    </model>
    <model name='grey_wall_1'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>7.47989 -6.6141 0 0 -0 0</pose>
    </model>
    <model name='grey_wall_clone_clone_clone_clone_clone'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>7.47099 -0.378407 0 0 0 -1.55997</pose>
    </model>
    <model name='grey_wall_clone_clone_clone_clone_clone_0'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>-2.39974 -8.81146 0 0 0 -1.55997</pose>
    </model>
    <model name='grey_wall_0_clone'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>1.27132 -12.4561 0 0 -0 0</pose>
    </model>
    <model name='grey_wall_clone_clone_clone_clone_clone_0_clone'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>4.90981 -10.2985 0 0 0 -1.55997</pose>
    </model>
    <model name='grey_wall_0_clone_clone'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>3.76832 -13.9698 0 0 -0 0</pose>
    </model>
    <model name='grey_wall_1_clone_clone'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>11.1202 4.84283 0 0 -0 0</pose>
    </model>
    <model name='grey_wall_1_clone_clone_0'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>15.9169 -3.87949 0 0 -0 0</pose>
    </model>
    <model name='grey_wall_clone_clone_clone_clone_clone_0_clone_clone'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>14.6322 -10.2305 0 0 0 -1.55997</pose>
    </model>
    <model name='grey_wall_clone_clone_clone_clone_clone_clone'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>14.8032 1.13647 0 0 0 -1.55997</pose>
    </model>
    <model name='grey_wall_clone_clone_clone_clone_clone_clone_0'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>14.5781 -5.50863 0 0 0 -1.55997</pose>
    </model>
    <model name='fence'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 0.915 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.1 5 1.83</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>0.1 5 1.83</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://fence/materials/scripts</uri>
              <uri>model://fence/materials/textures</uri>
              <name>Fence/WoodBoarding</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>-9.96032 3.22519 0 0 -0 0</pose>
    </model>
    <model name='fence_0'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 0.915 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.1 5 1.83</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>0.1 5 1.83</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://fence/materials/scripts</uri>
              <uri>model://fence/materials/textures</uri>
              <name>Fence/WoodBoarding</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>-9.92653 8.19667 0 0 -0 0</pose>
    </model>
    <model name='fence_1'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 0.915 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.1 5 1.83</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>0.1 5 1.83</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://fence/materials/scripts</uri>
              <uri>model://fence/materials/textures</uri>
              <name>Fence/WoodBoarding</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>-7.4543 10.6691 0 0 -0 0</pose>
    </model>
    <model name='fence_1_clone'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 0.915 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.1 5 1.83</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>0.1 5 1.83</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://fence/materials/scripts</uri>
              <uri>model://fence/materials/textures</uri>
              <name>Fence/WoodBoarding</name>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>-2.66827 10.027 0 0 0 -1.56912</pose>
    </model>
    <model name='Pine Tree'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>model://Pine Tree/meshes/pine_tree.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='branch'>
          <geometry>
            <mesh>
              <uri>model://Pine Tree/meshes/pine_tree.dae</uri>
              <submesh>
                <name>Branch</name>
              </submesh>
            </mesh>
          </geometry>
          <material>
            <double_sided>1</double_sided>
            <diffuse>1 1 1 1</diffuse>
            <script>
              <uri>model://Pine Tree/materials/scripts/</uri>
              <uri>model://Pine Tree/materials/textures/</uri>
              <name>PineTree/Branch</name>
            </script>
            <pbr>
              <metal>
                <albedo_map>model://Pine Tree/materials/textures/branch_2_diffuse.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
        <visual name='bark'>
          <geometry>
            <mesh>
              <uri>model://Pine Tree/meshes/pine_tree.dae</uri>
              <submesh>
                <name>Bark</name>
              </submesh>
            </mesh>
          </geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <script>
              <uri>model://Pine Tree/materials/scripts/</uri>
              <uri>model://Pine Tree/materials/textures/</uri>
              <name>PineTree/Bark</name>
            </script>
            <pbr>
              <metal>
                <albedo_map>model://Pine Tree/materials/textures/bark_diffuse.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>-8.94929 2.07668 0 0 -0 0</pose>
    </model>
    <model name='Pine Tree_0'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>model://Pine Tree/meshes/pine_tree.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='branch'>
          <geometry>
            <mesh>
              <uri>model://Pine Tree/meshes/pine_tree.dae</uri>
              <submesh>
                <name>Branch</name>
              </submesh>
            </mesh>
          </geometry>
          <material>
            <double_sided>1</double_sided>
            <diffuse>1 1 1 1</diffuse>
            <script>
              <uri>model://Pine Tree/materials/scripts/</uri>
              <uri>model://Pine Tree/materials/textures/</uri>
              <name>PineTree/Branch</name>
            </script>
            <pbr>
              <metal>
                <albedo_map>model://Pine Tree/materials/textures/branch_2_diffuse.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
        <visual name='bark'>
          <geometry>
            <mesh>
              <uri>model://Pine Tree/meshes/pine_tree.dae</uri>
              <submesh>
                <name>Bark</name>
              </submesh>
            </mesh>
          </geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <script>
              <uri>model://Pine Tree/materials/scripts/</uri>
              <uri>model://Pine Tree/materials/textures/</uri>
              <name>PineTree/Bark</name>
            </script>
            <pbr>
              <metal>
                <albedo_map>model://Pine Tree/materials/textures/bark_diffuse.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>-9.01496 3.97795 0 0 -0 0</pose>
    </model>
    <model name='Pine Tree_1'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>model://Pine Tree/meshes/pine_tree.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='branch'>
          <geometry>
            <mesh>
              <uri>model://Pine Tree/meshes/pine_tree.dae</uri>
              <submesh>
                <name>Branch</name>
              </submesh>
            </mesh>
          </geometry>
          <material>
            <double_sided>1</double_sided>
            <diffuse>1 1 1 1</diffuse>
            <script>
              <uri>model://Pine Tree/materials/scripts/</uri>
              <uri>model://Pine Tree/materials/textures/</uri>
              <name>PineTree/Branch</name>
            </script>
            <pbr>
              <metal>
                <albedo_map>model://Pine Tree/materials/textures/branch_2_diffuse.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
        <visual name='bark'>
          <geometry>
            <mesh>
              <uri>model://Pine Tree/meshes/pine_tree.dae</uri>
              <submesh>
                <name>Bark</name>
              </submesh>
            </mesh>
          </geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <script>
              <uri>model://Pine Tree/materials/scripts/</uri>
              <uri>model://Pine Tree/materials/textures/</uri>
              <name>PineTree/Bark</name>
            </script>
            <pbr>
              <metal>
                <albedo_map>model://Pine Tree/materials/textures/bark_diffuse.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>-7.44565 1.90429 0 0 -0 0</pose>
    </model>
    <state world_name='default'>
      <sim_time>5118 367000000</sim_time>
      <real_time>111 624840470</real_time>
      <wall_time>1733487393 804409985</wall_time>
      <iterations>109001</iterations>
      <model name='PatientWheelChair'>
        <pose>-1.08886 -12.0418 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='body'>
          <pose>-1.08886 -12.0418 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='Pine Tree'>
        <pose>-8.94929 2.07668 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-8.94929 2.07668 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='Pine Tree_0'>
        <pose>-9.01496 3.97795 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-9.01496 3.97795 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='Pine Tree_0_clone'>
        <pose>-3.73272 8.74553 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-3.73272 8.74553 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='Pine Tree_0_clone_0'>
        <pose>-1.64177 8.67523 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-1.64177 8.67523 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='Pine Tree_1'>
        <pose>-7.19572 2.06359 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-7.19572 2.06359 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='Pine Tree_1_clone'>
        <pose>-1.66718 6.51352 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-1.66718 6.51352 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='blue_block'>
        <pose>11.9469 -10.4967 0.276983 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>11.9469 -10.4967 0.276983 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='fence'>
        <pose>-9.96032 3.22519 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-9.96032 3.22519 0.915 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='fence_0'>
        <pose>-9.92653 7.49412 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-9.92653 7.49412 0.915 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='fence_1'>
        <pose>-7.48576 10.0176 0 0 0 -1.56912</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-7.48576 10.0176 0.915 0 0 -1.56912</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='fence_1_clone'>
        <pose>-2.66827 10.027 0 0 0 -1.56912</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-2.66827 10.027 0.915 0 0 -1.56912</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='green_block'>
        <pose>-5.70129 4.69308 0.279022 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-5.70129 4.69308 0.279022 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall'>
        <pose>3.74092 8.10152 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>3.74092 8.10152 1.4 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_0'>
        <pose>-2.3204 -6.67079 0 0 0 -0.007982</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-2.3204 -6.67079 1.4 0 0 -0.007982</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_0_clone'>
        <pose>1.3111 -13.9413 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>1.3111 -13.9413 1.4 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_0_clone_clone'>
        <pose>3.76832 -13.9698 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>3.76832 -13.9698 1.4 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_1'>
        <pose>7.47989 -6.6141 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>7.47989 -6.6141 1.4 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_1_clone_clone'>
        <pose>11.1202 1.38938 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>11.1202 1.38938 1.4 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_1_clone_clone_0'>
        <pose>11.266 -13.9767 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>11.266 -13.9767 1.4 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_clone'>
        <pose>7.42476 4.45325 0 0 0 -1.55997</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>7.42476 4.45325 1.4 0 0 -1.55997</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_clone_0'>
        <pose>1.21826 0.772472 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>1.21826 0.772472 1.4 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_clone_0_clone'>
        <pose>-6.25143 0.766627 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-6.25143 0.766627 1.4 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_clone_0_clone_clone'>
        <pose>-6.18838 -6.64123 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-6.18838 -6.64123 1.4 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_clone_clone'>
        <pose>0.00868 6.48654 0 0 0 -1.55997</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0.00868 6.48654 1.4 0 0 -1.55997</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_clone_clone_clone'>
        <pose>-9.87851 -3.01561 0 0 0 -1.55997</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-9.87851 -3.01561 1.4 0 0 -1.55997</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_clone_clone_clone_clone'>
        <pose>-2.44882 -5.0918 0 0 0 -1.55997</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-2.44882 -5.0918 1.4 0 0 -1.55997</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_clone_clone_clone_clone_clone'>
        <pose>7.47099 -0.378407 0 0 0 -1.55997</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>7.47099 -0.378407 1.4 0 0 -1.55997</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_clone_clone_clone_clone_clone_0'>
        <pose>-2.38426 -10.2409 0 0 0 -1.55997</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-2.38426 -10.2409 1.4 0 0 -1.55997</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_clone_clone_clone_clone_clone_0_clone'>
        <pose>7.34574 -10.2721 0 0 0 -1.55997</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>7.34574 -10.2721 1.4 0 0 -1.55997</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_clone_clone_clone_clone_clone_0_clone_clone'>
        <pose>14.9132 -10.2275 0 0 0 -1.55997</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>14.9132 -10.2275 1.4 0 0 -1.55997</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_clone_clone_clone_clone_clone_clone'>
        <pose>14.8258 -2.28049 0 0 0 -1.55997</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>14.8258 -2.28049 1.4 0 0 -1.55997</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='grey_wall_clone_clone_clone_clone_clone_clone_0'>
        <pose>14.8771 -5.46328 0 0 0 -1.55997</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>14.8771 -5.46328 1.4 0 0 -1.55997</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='ground_plane'>
        <pose>0 0 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='red_block'>
        <pose>1.10744 -10.647 0.270807 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>1.10744 -10.647 0.270807 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='red_block_0'>
        <pose>-6.24875 -3.20938 0.27733 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-6.24875 -3.20938 0.27733 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box'>
        <pose>3.66894 -11.713 0.298088 0 7.9e-05 0</pose>
        <scale>15.0258 56.4923 0.234477</scale>
        <link name='link'>
          <pose>3.66894 -11.713 0.298088 0 7.9e-05 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_0'>
        <pose>1.74802 -13.2296 0.375865 0 -0 0</pose>
        <scale>0.630153 0.540994 0.37725</scale>
        <link name='link'>
          <pose>1.74802 -13.2296 0.375865 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_0_clone'>
        <pose>9.14855 0.861617 0.375865 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>9.14855 0.861617 0.375865 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_0_clone_0'>
        <pose>7.9175 -11.3747 0.375865 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>7.9175 -11.3747 0.375865 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_0_clone_0_clone'>
        <pose>-9.37263 -4.33858 0.375865 0 0 -0.000241</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-9.37263 -4.33858 0.375865 0 0 -0.000241</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_0_clone_1'>
        <pose>8.61511 -13.3636 0.650681 -5e-06 -3e-06 -2.8e-05</pose>
        <scale>1 1 7.05887</scale>
        <link name='link'>
          <pose>8.61511 -13.3636 0.650681 -5e-06 -3e-06 -2.8e-05</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_0_clone_1_clone'>
        <pose>10.8351 -13.591 1.0308 0 1e-06 0.000962</pose>
        <scale>2.75976 0.040437 2.92573</scale>
        <link name='link'>
          <pose>10.8351 -13.591 1.0308 0 1e-06 0.000962</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_0_clone_1_clone_0'>
        <pose>-8.43999 -6.09933 0.658153 1e-06 -1e-06 -0.000394</pose>
        <scale>1 1 0.453839</scale>
        <link name='link'>
          <pose>-8.43999 -6.09933 0.658153 1e-06 -1e-06 -0.000394</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_0_clone_1_clone_clone'>
        <pose>-6.52494 -6.3723 1.13043 0 -2e-06 -0.000174</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-6.52494 -6.3723 1.13043 0 -2e-06 -0.000174</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_0_clone_2'>
        <pose>-0.521787 -5.57661 0.181331 0.003265 2.2e-05 -0.00016</pose>
        <scale>2.2345 3.33156 0.103604</scale>
        <link name='link'>
          <pose>-0.521787 -5.57661 0.181331 0.003265 2.2e-05 -0.00016</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_0_clone_2_clone'>
        <pose>-0.517257 -6.34526 0.51895 1.57404 4.7e-05 -0.007982</pose>
        <scale>0.940755 0.113287 8.42548</scale>
        <link name='link'>
          <pose>-0.517257 -6.34526 0.51895 1.57404 4.7e-05 -0.007982</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_0_clone_2_clone_0'>
        <pose>-1.35748 -3.01592 0.183335 0 -0 0.001113</pose>
        <scale>1 4.32442 1</scale>
        <link name='link'>
          <pose>-1.35748 -3.01592 0.183335 0 -0 0.001113</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_0_clone_2_clone_0_clone'>
        <pose>-2.15215 -3.03182 0.515239 -0.487223 -1.57079 0.487084</pose>
        <scale>1 0.915232 1</scale>
        <link name='link'>
          <pose>-2.15215 -3.03182 0.515239 -0.487223 -1.57079 0.487084</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_1'>
        <pose>1.70188 7.45396 0.499998 4e-06 0 0.024736</pose>
        <scale>316.008 1 0.999426</scale>
        <link name='link'>
          <pose>1.70188 7.45396 0.499998 4e-06 0 0.024736</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_1_clone'>
        <pose>0.630881 5.34421 0.499713 0 0 -1.55561</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0.630881 5.34421 0.499713 0 0 -1.55561</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_1_clone_0'>
        <pose>3.80089 7.48938 1.10887 -1.61512 -1.57079 0.068762</pose>
        <scale>0.451934 1 1</scale>
        <link name='link'>
          <pose>3.80089 7.48938 1.10887 -1.61512 -1.57079 0.068762</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_1_clone_1'>
        <pose>5.78814 7.46198 0.499713 0 -0 0.013851</pose>
        <scale>0.735051 1 1</scale>
        <link name='link'>
          <pose>5.78814 7.46198 0.499713 0 -0 0.013851</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_clone'>
        <pose>6.89495 -10.7187 1.33123 -1.5708 -3e-06 1.62263</pose>
        <scale>1 0.3742 1</scale>
        <link name='link'>
          <pose>6.89495 -10.7187 1.33123 -1.5708 -3e-06 1.62263</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_clone_0'>
        <pose>10.9909 -0.646617 0.298088 0 -7.9e-05 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>10.9909 -0.646617 0.298088 0 -7.9e-05 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_clone_1'>
        <pose>8.69087 -9.54498 0.212832 9e-06 -0 0.001247</pose>
        <scale>1 0.029912 0.662556</scale>
        <link name='link'>
          <pose>8.69087 -9.54498 0.212832 9e-06 -0 0.001247</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_clone_1_clone'>
        <pose>10.8356 -12.9304 0.114866 0 0 -0.008909</pose>
        <scale>0.094162 0.900944 0.042587</scale>
        <link name='link'>
          <pose>10.8356 -12.9304 0.114866 0 0 -0.008909</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_clone_1_clone_0'>
        <pose>-8.55818 -2.51831 0.298179 -9e-06 -0 0.01115</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-8.55818 -2.51831 0.298179 -9e-06 -0 0.01115</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_clone_1_clone_0_clone'>
        <pose>-5.59385 6.36468 0.298178 1.1e-05 -0 0.011005</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-5.59385 6.36468 0.298178 1.1e-05 -0 0.011005</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_clone_1_clone_clone'>
        <pose>-6.53327 -5.74314 0.114866 0 -0 4e-05</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-6.53327 -5.74314 0.114866 0 -0 4e-05</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_clone_2'>
        <pose>2.56022 -3.50146 0.298088 0 -7.9e-05 0</pose>
        <scale>1 0.163357 1</scale>
        <link name='link'>
          <pose>2.56022 -3.50146 0.298088 0 -7.9e-05 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box_clone_clone'>
        <pose>14.3149 -1.73383 1.33123 -1.5708 -0 1.58865</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>14.3149 -1.73383 1.33123 -1.5708 -0 1.58865</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='yellow_block'>
        <pose>3.97374 5.3705 0.274114 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>3.97374 5.3705 0.274114 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <light name='sun'>
        <pose>0 0 10 0 -0 0</pose>
      </light>
    </state>
    <model name='PatientWheelChair'>
      <static>1</static>
      <link name='body'>
        <pose>0 0 0 0 -0 0</pose>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://PatientWheelChair/meshes/PatientWheelChair.obj</uri>
            </mesh>
          </geometry>
        </visual>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>model://PatientWheelChair/meshes/PatientWheelChair_Col.obj</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>-1.08886 -12.0418 0 0 -0 0</pose>
    </model>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>20.7612 5.61455 13.7877 0 0.6698 -2.37498</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
    <model name='unit_box'>
      <pose>2.71201 -5.18625 0.5 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.166701 0.069591 2.81172</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.166701 0.069591 2.81172</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_clone'>
      <pose>5.43251 -3.44834 0.5 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>2.44924 7.68415 0.596368</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>2.44924 7.68415 0.596368</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_0'>
      <pose>1.74802 -13.2296 0.5 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>1.67026 1.90035 1.76961</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>1.67026 1.90035 1.76961</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_clone_0'>
      <pose>11.8659 -0.646632 0.322333 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>2.44924 3.79071 0.596368</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>2.44924 3.79071 0.596368</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_clone_clone'>
      <pose>14.2094 -1.76936 1.36855 -1.57404 -0.022616 1.61855</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>2.44924 2.66247 0.596368</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>2.44924 2.66247 0.596368</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_0_clone'>
      <pose>9.96198 0.861617 0.370369 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.773763 0.725409 0.75173</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.773763 0.725409 0.75173</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_clone_1'>
      <pose>12.5459 -8.07757 0.322333 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>2.44924 44.9863 0.596368</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>2.44924 44.9863 0.596368</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_0_clone_0'>
      <pose>12.5578 -10.0631 0.370369 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.773763 0.725409 0.75173</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.773763 0.725409 0.75173</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_0_clone_1'>
      <pose>13.3476 -11.5495 0.370369 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.773763 0.725409 0.175514</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.773763 0.725409 0.175514</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_clone_1_clone'>
      <pose>12.0177 -5.57867 0.322333 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>10.357 1.10037 4.01883</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>10.357 1.10037 4.01883</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_clone_1_clone_0'>
      <pose>-8.67591 -2.52847 0.322333 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>2.44924 1.10037 0.596368</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>2.44924 1.10037 0.596368</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_0_clone_0_clone'>
      <pose>-9.38365 -4.33876 0.370369 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.773763 0.725409 0.75173</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.773763 0.725409 0.75173</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_0_clone_1_clone_0'>
      <pose>-8.4401 -6.18184 0.370369 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.773763 0.725409 2.74467</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.773763 0.725409 2.74467</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_clone_1_clone_clone'>
      <pose>-6.53327 -5.97418 0.154826 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>1.19105 1.10037 0.229732</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>1.19105 1.10037 0.229732</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_0_clone_1_clone_clone'>
      <pose>-6.525 -6.34813 1.22492 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>1.04986 0.148665 2.26087</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>1.04986 0.148665 2.26087</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_0_clone_2'>
      <pose>2.46312 -5.66089 0.375865 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.436257 0.34271 3.15961</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.436257 0.34271 3.15961</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_0_clone_2_clone'>
      <pose>0.883428 -5.34019 0.183335 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>1.16436 11.9513 0.040398</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>1.16436 11.9513 0.040398</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_0_clone_2_clone_0'>
      <pose>-0.457458 -3.70447 0.183335 0 -0 8.1e-05</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>1.03048 0.399251 0.366671</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>1.03048 0.399251 0.366671</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_0_clone_2_clone_0_clone'>
      <pose>-0.306983 -3.58458 0.183335 0 -0 8.1e-05</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>1.03048 2.04845 0.366671</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>1.03048 2.04845 0.366671</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='Pine Tree_0_clone'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>model://Pine Tree/meshes/pine_tree.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='branch'>
          <geometry>
            <mesh>
              <uri>model://Pine Tree/meshes/pine_tree.dae</uri>
              <submesh>
                <name>Branch</name>
              </submesh>
            </mesh>
          </geometry>
          <material>
            <double_sided>1</double_sided>
            <diffuse>1 1 1 1</diffuse>
            <script>
              <uri>model://Pine Tree/materials/scripts/</uri>
              <uri>model://Pine Tree/materials/textures/</uri>
              <name>PineTree/Branch</name>
            </script>
            <pbr>
              <metal>
                <albedo_map>model://Pine Tree/materials/textures/branch_2_diffuse.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
        <visual name='bark'>
          <geometry>
            <mesh>
              <uri>model://Pine Tree/meshes/pine_tree.dae</uri>
              <submesh>
                <name>Bark</name>
              </submesh>
            </mesh>
          </geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <script>
              <uri>model://Pine Tree/materials/scripts/</uri>
              <uri>model://Pine Tree/materials/textures/</uri>
              <name>PineTree/Bark</name>
            </script>
            <pbr>
              <metal>
                <albedo_map>model://Pine Tree/materials/textures/bark_diffuse.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>-4.61475 9.04598 0 0 -0 0</pose>
    </model>
    <model name='Pine Tree_0_clone_0'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>model://Pine Tree/meshes/pine_tree.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='branch'>
          <geometry>
            <mesh>
              <uri>model://Pine Tree/meshes/pine_tree.dae</uri>
              <submesh>
                <name>Branch</name>
              </submesh>
            </mesh>
          </geometry>
          <material>
            <double_sided>1</double_sided>
            <diffuse>1 1 1 1</diffuse>
            <script>
              <uri>model://Pine Tree/materials/scripts/</uri>
              <uri>model://Pine Tree/materials/textures/</uri>
              <name>PineTree/Branch</name>
            </script>
            <pbr>
              <metal>
                <albedo_map>model://Pine Tree/materials/textures/branch_2_diffuse.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
        <visual name='bark'>
          <geometry>
            <mesh>
              <uri>model://Pine Tree/meshes/pine_tree.dae</uri>
              <submesh>
                <name>Bark</name>
              </submesh>
            </mesh>
          </geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <script>
              <uri>model://Pine Tree/materials/scripts/</uri>
              <uri>model://Pine Tree/materials/textures/</uri>
              <name>PineTree/Bark</name>
            </script>
            <pbr>
              <metal>
                <albedo_map>model://Pine Tree/materials/textures/bark_diffuse.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>-1.30377 8.67523 0 0 -0 0</pose>
    </model>
    <model name='Pine Tree_1_clone'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>model://Pine Tree/meshes/pine_tree.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='branch'>
          <geometry>
            <mesh>
              <uri>model://Pine Tree/meshes/pine_tree.dae</uri>
              <submesh>
                <name>Branch</name>
              </submesh>
            </mesh>
          </geometry>
          <material>
            <double_sided>1</double_sided>
            <diffuse>1 1 1 1</diffuse>
            <script>
              <uri>model://Pine Tree/materials/scripts/</uri>
              <uri>model://Pine Tree/materials/textures/</uri>
              <name>PineTree/Branch</name>
            </script>
            <pbr>
              <metal>
                <albedo_map>model://Pine Tree/materials/textures/branch_2_diffuse.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
        <visual name='bark'>
          <geometry>
            <mesh>
              <uri>model://Pine Tree/meshes/pine_tree.dae</uri>
              <submesh>
                <name>Bark</name>
              </submesh>
            </mesh>
          </geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <script>
              <uri>model://Pine Tree/materials/scripts/</uri>
              <uri>model://Pine Tree/materials/textures/</uri>
              <name>PineTree/Bark</name>
            </script>
            <pbr>
              <metal>
                <albedo_map>model://Pine Tree/materials/textures/bark_diffuse.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
      <pose>-1.25254 6.51114 0 0 -0 0</pose>
    </model>
    <model name='unit_box_1'>
      <pose>1.26726 5.03489 0.5 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.009864 1 1.00114</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.009864 1 1.00114</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_1_clone'>
      <pose>1.94613 5.78557 0.499485 0 -0.000145 0.000102</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>3.15321 1 0.999426</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>3.15321 1 0.999426</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_1_clone_0'>
      <pose>3.39938 5.1911 0.499485 0 0.000145 -5.1e-05</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>4.81577 1 0.999426</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>4.81577 1 0.999426</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_1_clone_1'>
      <pose>5.64084 5.62739 0.49971 5e-06 -0 0.027926</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>3.88185 1 0.999426</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>3.88185 1 0.999426</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_0_clone_1_clone'>
      <pose>12.2349 -12.6039 0.777871 0 -5e-06 -0.000236</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.399042 6.24975 0.946198</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.399042 6.24975 0.946198</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_clone_2'>
      <pose>2.37592 -3.01325 0.298184 0 1e-06 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>2.44924 14.1355 0.596368</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>2.44924 14.1355 0.596368</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='unit_box_clone_1_clone_0_clone'>
      <pose>-5.59384 6.36496 0.298179 9e-06 -0 0.011033</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>2.44924 1.10037 0.596368</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>2.44924 1.10037 0.596368</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>1</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>1</kinematic>
      </link>
    </model>
    <model name='red_block'>
      <static>1</static>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>1.10744 -10.647 0 0 -0 0</pose>
    </model>
    <model name='blue_block'>
      <static>1</static>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>11.9469 -10.4967 0 0 -0 0</pose>
    </model>
    <model name='yellow_block'>
      <static>1</static>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>1 1 0 1</ambient>
            <diffuse>1 1 0 1</diffuse>
          </material>
        </visual>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>3.97374 5.3705 0 0 -0 0</pose>
    </model>
    <model name='green_block'>
      <static>1</static>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>-5.70129 4.69308 0 0 -0 0</pose>
    </model>
    <model name='red_block_0'>
      <static>1</static>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>-6.24875 -3.20938 0 0 -0 0</pose>
    </model>
  </world>
</sdf>
