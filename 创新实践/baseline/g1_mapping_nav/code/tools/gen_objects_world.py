import os

WS = "/home/wy/go2_sim_ws/ROS2-Gazebo-GO2"
SRC = WS + "/src/gazebo_sim"
base = open(SRC + "/world/warehouse_cross.sdf").read()


def actor_person(name, pts, speed=0.3):
    x0, y0 = pts[0]
    # cumulative distance/timings (relative waypoints)
    import math
    t = 0.0
    wps = []
    for i, (x, y) in enumerate(pts):
        if i > 0:
            t += math.hypot(x - pts[i - 1][0], y - pts[i - 1][1]) / speed
        wps.append((round(t, 3), round(x - x0, 4), round(y - y0, 4)))
    body = "".join(
        f"          <waypoint><time>{tt}</time><pose>{dx} {dy} 0 0 0 0</pose></waypoint>\n"
        for tt, dx, dy in wps)
    body += f"          <waypoint><time>{round(t + math.hypot(pts[0][0]-pts[-1][0], pts[0][1]-pts[-1][1])/speed,3)}</time><pose>0 0 0 0 0 0</pose></waypoint>\n"
    return f'''
    <actor name="{name}">
      <pose>{x0} {y0} 0 0 0 0</pose>
      <skin><filename>model://walking_actor/meshes/walk.dae</filename><scale>1.0</scale></skin>
      <animation name="walk"><filename>model://walking_actor/meshes/walk.dae</filename><scale>1.0</scale><interpolate_x>true</interpolate_x></animation>
      <script>
        <loop>true</loop><auto_start>true</auto_start>
        <trajectory id="0" type="walk" tension="1.0">
{body}        </trajectory>
      </script>
    </actor>
'''


def car_model(name, wps, speed=0.5):
    wp = "".join(f"\n        <waypoint>{x} {y} 0</waypoint>" for x, y in wps)
    x0, y0 = wps[0]
    return f'''
    <model name="{name}">
      <pose>{x0} {y0} 0.06 0 0 0</pose>
      <link name="link">
        <inertial><pose>0 0 0.7 0 0 0</pose><mass>1200</mass>
          <inertia><ixx>600</ixx><ixy>0</ixy><ixz>0</ixz><iyy>2500</iyy><iyz>0</iyz><izz>2800</izz></inertia></inertial>
        <collision name="collision"><pose>0 0 0.7 0 0 0</pose>
          <geometry><box><size>3.0 1.6 1.4</size></box></geometry></collision>
        <visual name="visual"><pose>0 0 0 0 0 -1.57079632679</pose>
          <geometry><mesh><scale>0.04 0.04 0.04</scale><uri>model://suv_vehicle/meshes/suv.obj</uri></mesh></geometry></visual>
      </link>
      <plugin filename="libTrajDrive.so" name="traj_drive::TrajDrive">
        <link>link</link><speed>{speed}</speed><kp>2.2</kp><loop>true</loop><range_tolerance>0.4</range_tolerance>{wp}
      </plugin>
    </model>
'''


parts = ["\n    <!-- ==== 行人：<actor> 长走廊矩形环（物理无关，sim-time 平滑） ==== -->\n"]
parts.append(actor_person("person_1", [(1.0, -1.15), (11.0, -1.15), (11.0, 1.25), (1.0, 1.25)]))
parts.append(actor_person("person_2", [(1.0, 1.25), (11.0, 1.25), (11.0, 3.85), (1.0, 3.85)]))
parts.append(actor_person("person_3", [(1.0, 3.85), (11.0, 3.85), (11.0, 5.4), (1.0, 5.4)]))
parts.append("\n    <!-- ==== 车辆：动态刚体 + libTrajDrive.so（真碰撞，可被机器人推动） ==== -->\n")
parts.append(car_model("suv_1", [(-6.5, -6.45), (1.5, -6.45)]))
parts.append(car_model("suv_2", [(12.0, -6.45), (4.5, -6.45)]))

idx = base.rfind("</world>")
new = base[:idx] + "".join(parts) + base[idx:]
for p in [SRC + "/world/warehouse_cross_objects.sdf",
          WS + "/install/gazebo_sim/share/gazebo_sim/world/warehouse_cross_objects.sdf"]:
    os.makedirs(os.path.dirname(p), exist_ok=True)
    open(p, "w").write(new)
    print("wrote", p, len(new))
