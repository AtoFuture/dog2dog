"""Generate a private static world; production navigation receives only real sensors."""
import argparse
from pathlib import Path
from qwen_robotnav.static_scenarios import SCENARIOS, prepare, configure_spawn

if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--world',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--scenario',choices=SCENARIOS,required=True)
    a=p.parse_args()
    configure_spawn(a.output_dir/'go2_sensors_nav2.launch.py')
    print(prepare(a.world,a.output_dir,a.scenario))
