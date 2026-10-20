"""Detect source changes across sequential runtime experiments."""
import hashlib
from pathlib import Path


def runtime_sources(repository):
    repository = Path(repository)
    roots = [repository/'创新实践/baseline'/name for name in (
        'g1_mapping_nav/tools', 'g2_vision_explore', 'g3_integration_eval')]
    roots.append(repository/'创新实践/external/go2_real_robot/qwen_robotnav')
    result = {}
    for root in roots:
        for path in root.rglob('*'):
            if (not path.is_file() or (path.suffix not in ('.py','.sh','.yaml','.xml','.sdf','.cpp','.hpp')
                    and path.name != 'CMakeLists.txt')
                    or any(part in ('test','tests','__pycache__','.venv','build','install','log')
                           for part in path.relative_to(root).parts)):
                continue
            result[str(path.relative_to(repository))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return dict(sorted(result.items()))


def changed_sources(before, after):
    return sorted(name for name in before.keys() | after.keys()
                  if before.get(name) != after.get(name))
