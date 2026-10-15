"""The scoped Humble workaround must forward the timeout to the load RPC."""
from pathlib import Path
import runpy
import subprocess
import sys
import types


def test_generated_spawner_forwards_load_timeout_without_modifying_ros(tmp_path,monkeypatch):
    script=Path(__file__).resolve().parents[1]/'scripts/prepare_simulator_launch.py'
    launch=tmp_path/'sim/src/gazebo_sim/launch';launch.mkdir(parents=True)
    (launch/'gazebo_go2_sensors.launch.py').write_text("robot_desc = xacro.process_file(xacro_file, mappings={'robot_name': robot_name}).toxml()\nexecutable='spawner'\n")
    (launch/'launch.py').write_text("launch_file = 'gazebo_go2_sensors.launch.py'\n")
    output=tmp_path/'output'
    subprocess.run([sys.executable,str(script),'--sim-workspace',str(tmp_path/'sim'),'--output-dir',str(output)],check=True,capture_output=True)
    calls=[];package=types.ModuleType('controller_manager');spawner=types.ModuleType('controller_manager.spawner')
    services=types.ModuleType('controller_manager.controller_manager_services')
    services.load_controller=lambda *args,**kwargs:calls.append((args,kwargs))
    spawner.main=lambda:spawner.load_controller('node','manager','joint_controller')
    package.spawner=spawner
    for name,module in [('controller_manager',package),('controller_manager.spawner',spawner),('controller_manager.controller_manager_services',services)]:
        monkeypatch.setitem(sys.modules,name,module)
    runpy.run_path(str(output/'controller_spawner'))
    assert calls==[(('node','manager','joint_controller'),{'service_timeout':60.,'call_timeout':60.})]
    assert str(output/'controller_spawner') in (output/'go2_sensors_nav2.launch.py').read_text()
