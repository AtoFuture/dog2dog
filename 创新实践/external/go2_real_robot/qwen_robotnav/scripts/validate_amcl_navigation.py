"""Historical AMCL entry; default runtime and validators never publish velocity."""
import runpy
import sys
from pathlib import Path

if __name__=='__main__':
    if '--legacy-validation' not in sys.argv:
        raise SystemExit('Historical AMCL/visual experiment is archived; explicit --legacy-validation is required. Use run_runtime_trial.py for current acceptance.')
    runpy.run_path(str(Path(__file__).parent/'legacy/validate_amcl_navigation.py'),run_name='__main__')
