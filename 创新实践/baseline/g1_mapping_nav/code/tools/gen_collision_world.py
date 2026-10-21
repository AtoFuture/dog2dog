import json, os

WS = "/home/wy/go2_sim_ws/ROS2-Gazebo-GO2"
boxes = json.load(open('/tmp/col_boxes.json'))
# keep only inside the depot building envelope (exclude out-of-building artifacts)
boxes = [(x, y) for (x, y) in boxes if -8.6 <= x <= 22.6 and -7.3 <= y <= 8.8]
print('boxes kept:', len(boxes))

base = open(WS + "/src/gazebo_sim/world/warehouse_objects.sdf").read()
parts = ['\n    <!-- auto-generated collision for Depot VISUAL geometry (1m cells, z 0..3) -->\n',
         '    <model name="depot_visual_collision">\n      <static>1</static>\n      <link name="l">\n']
for i, (x, y) in enumerate(boxes):
    parts.append(
        f'        <collision name="c{i}"><pose>{x} {y} 1.5 0 0 0</pose>'
        f'<geometry><box><size>1 1 3</size></box></geometry></collision>\n')
parts.append('      </link>\n    </model>\n')

idx = base.rfind("</world>")
new = base[:idx] + "".join(parts) + base[idx:]
for p in [WS + "/src/gazebo_sim/world/warehouse_objects_col.sdf",
          WS + "/install/gazebo_sim/share/gazebo_sim/world/warehouse_objects_col.sdf"]:
    open(p, "w").write(new)
    print("wrote", p, len(new))
