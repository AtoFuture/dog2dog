import os, math

WS = "/home/wy/go2_sim_ws/ROS2-Gazebo-GO2"
SRC = WS + "/src/gazebo_sim"
base = open(SRC + "/world/warehouse_cross.sdf").read()


def actor_person(name, pts, speed=0.3):
    x0, y0 = pts[0]
    t = 0.0
    body = ""
    for i, (x, y) in enumerate(pts):
        if i > 0:
            t += math.hypot(x - pts[i - 1][0], y - pts[i - 1][1]) / speed
        body += (f"          <waypoint><time>{round(t,3)}</time>"
                 f"<pose>{round(x-x0,4)} {round(y-y0,4)} 0 0 0 0</pose></waypoint>\n")
    t += math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) / speed
    body += f"          <waypoint><time>{round(t,3)}</time><pose>0 0 0 0 0 0</pose></waypoint>\n"
    return f'''
    <actor name="{name}">
      <pose>{x0} {y0} 0 0 0 0</pose>
      <skin><filename>model://person_standing/meshes/standing.dae</filename><scale>1.0</scale></skin>
      <animation name="walk"><filename>model://walking_actor/meshes/walk.dae</filename><scale>1.0</scale><interpolate_x>true</interpolate_x></animation>
      <script>
        <loop>true</loop><auto_start>true</auto_start>
        <trajectory id="0" type="walk" tension="1.0">
{body}        </trajectory>
      </script>
    </actor>
'''


def wheel(name, x, y):
    return f'''
      <link name='{name}'>
        <pose>{x} {y} -0.025 -1.5707 0 0</pose>
        <inertial><mass>2</mass><inertia>
          <ixx>0.145833</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.145833</iyy><iyz>0</iyz><izz>0.125</izz></inertia></inertial>
        <visual name='visual'><geometry><cylinder><length>0.15</length><radius>0.3</radius></cylinder></geometry>
          <material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>
        <collision name='collision'><geometry><cylinder><length>0.15</length><radius>0.3</radius></cylinder></geometry>
          <surface><friction><ode><mu>0.5</mu><mu2>1.0</mu2><fdir1>0 0 1</fdir1></ode></friction></surface></collision>
      </link>
'''


def steer_link(name, y):
    return f'''
      <link name='{name}'>
        <pose>0.554283 {y} 0.02 0 0 0</pose>
        <inertial><mass>0.5</mass><inertia>
          <ixx>0.0153</ixx><iyy>0.025</iyy><izz>0.0153</izz></inertia></inertial>
        <visual name='v'><geometry><cylinder><length>0.1</length><radius>0.03</radius></cylinder></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual>
      </link>
'''


def vehicle(name, x, y, yaw, wps, speed):
    wp = "".join(f"\n        <waypoint>{ux} {uy} 0</waypoint>" for ux, uy in wps)
    return f'''
    <model name='{name}'>
      <pose>{x} {y} 0.325 0 0 {yaw}</pose>
      <link name='chassis'>
        <pose>-0.151427 0 0.175 0 0 0</pose>
        <inertial><mass>1.14395</mass><inertia>
          <ixx>0.126164</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.416519</iyy><iyz>0</iyz><izz>0.481014</izz></inertia></inertial>
        <visual name='visual'><geometry><box><size>2.01142 1 0.568726</size></box></geometry>
          <material><ambient>0.55 0.57 0.60 1</ambient><diffuse>0.55 0.57 0.60 1</diffuse>
          <specular>0.9 0.9 0.9 1</specular></material></visual>
        <collision name='collision'><geometry><box><size>2.01142 1 0.568726</size></box></geometry></collision>
      </link>
{wheel("front_left_wheel", 0.554283, 0.625029)}
{wheel("front_right_wheel", 0.554282, -0.625029)}
{wheel("rear_left_wheel", -0.957138, 0.625029)}
{wheel("rear_right_wheel", -0.957138, -0.625029)}
{steer_link("front_left_wheel_steering_link", 0.5)}
{steer_link("front_right_wheel_steering_link", -0.5)}
      <joint name='front_left_wheel_steering_joint' type='revolute'>
        <child>front_left_wheel_steering_link</child><parent>chassis</parent>
        <axis><xyz>0 0 1</xyz><limit><lower>-0.6</lower><upper>0.6</upper><velocity>1.0</velocity><effort>25</effort></limit><use_parent_model_frame>1</use_parent_model_frame></axis>
      </joint>
      <joint name='front_right_wheel_steering_joint' type='revolute'>
        <parent>chassis</parent><child>front_right_wheel_steering_link</child>
        <axis><xyz>0 0 1</xyz><limit><lower>-0.6</lower><upper>0.6</upper><velocity>1.0</velocity><effort>25</effort></limit></axis>
      </joint>
      <joint name='front_left_wheel_joint' type='revolute'>
        <parent>front_left_wheel_steering_link</parent><child>front_left_wheel</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>
      <joint name='front_right_wheel_joint' type='revolute'>
        <parent>front_right_wheel_steering_link</parent><child>front_right_wheel</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>
      <joint name='rear_left_wheel_joint' type='revolute'>
        <parent>chassis</parent><child>rear_left_wheel</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>
      <joint name='rear_right_wheel_joint' type='revolute'>
        <parent>chassis</parent><child>rear_right_wheel</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>

      <plugin filename="ignition-gazebo-ackermann-steering-system" name="gz::sim::systems::AckermannSteering">
        <left_joint>front_left_wheel_joint</left_joint>
        <left_joint>rear_left_wheel_joint</left_joint>
        <right_joint>front_right_wheel_joint</right_joint>
        <right_joint>rear_right_wheel_joint</right_joint>
        <left_steering_joint>front_left_wheel_steering_joint</left_steering_joint>
        <right_steering_joint>front_right_wheel_steering_joint</right_steering_joint>
        <kingpin_width>1.0</kingpin_width>
        <steering_limit>0.5</steering_limit>
        <wheel_base>1.0</wheel_base>
        <wheel_separation>1.25</wheel_separation>
        <wheel_radius>0.3</wheel_radius>
        <min_velocity>-1</min_velocity>
        <max_velocity>1</max_velocity>
        <min_acceleration>-3</min_acceleration>
        <max_acceleration>3</max_acceleration>
      </plugin>
      <plugin filename="libTrajDrive.so" name="traj_drive::TrajDrive">
        <link>chassis</link><speed>{speed}</speed><kp>1.5</kp><lookahead>2.0</lookahead><loop>true</loop>{wp}
      </plugin>
    </model>
'''


parts = ["\n    <!-- ==== 行人：<actor> 长走廊矩形环 ==== -->\n"]
parts.append(actor_person("person_1", [(10.8, 0.4), (10.8, 7.4)]))
parts.append(actor_person("person_2", [(18.75, 0.4), (18.75, 7.4)]))
parts.append(actor_person("person_3", [(8.0, 0.25), (20.5, 0.25)]))
parts.append(actor_person("person_4", [(14.65, -0.4), (14.65, 3.8)]))
parts.append(actor_person("person_5", [(8.0, -5.1), (20.0, -5.1)]))
parts.append("\n    <!-- ==== 车辆：带轮整车 + 官方 AckermannSteering + TrajDrive(cmd_vel) ==== -->\n")
parts.append(vehicle("suv_1", 3.6, -5.4, 1.5708, [(3.6, -5.4), (3.6, 7.0)], 0.6))
parts.append(vehicle("suv_2", 5.4, -5.4, 1.5708, [(5.4, -5.4), (5.4, 4.0)], 0.6))

idx = base.rfind("</world>")
new = base[:idx] + "".join(parts) + base[idx:]
for p in [SRC + "/world/warehouse_cross_objects.sdf",
          WS + "/install/gazebo_sim/share/gazebo_sim/world/warehouse_cross_objects.sdf"]:
    os.makedirs(os.path.dirname(p), exist_ok=True)
    open(p, "w").write(new)
    print("wrote", p, len(new))
