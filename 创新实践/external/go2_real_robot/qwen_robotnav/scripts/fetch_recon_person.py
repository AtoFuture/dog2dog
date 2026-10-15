"""Fetch the pinned public CC0 person asset; verify every cached/downloaded file."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess


def fetch(directory):
    directory=Path(directory).resolve();directory.mkdir(parents=True,exist_ok=True)
    manifest=json.loads(Path(__file__).with_name('recon_person_asset.json').read_text())
    base=manifest['source']+'/'+str(manifest['version'])+'/files'
    def one(item):
        path=(directory/item['path'].lstrip('/')).resolve()
        if not path.is_relative_to(directory):raise ValueError('Invalid manifest path')
        if path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest()==item['sha256']:return
        path.parent.mkdir(parents=True,exist_ok=True)
        temporary=path.with_suffix(path.suffix+'.download')
        try:
            subprocess.run(['curl','-fsSL','--max-time','300',base+item['path'],'-o',str(temporary)],check=True)
            if hashlib.sha256(temporary.read_bytes()).hexdigest()!=item['sha256']:
                raise ValueError('Asset checksum mismatch: '+item['path'])
            temporary.replace(path)
        finally:temporary.unlink(missing_ok=True)
    with ThreadPoolExecutor(max_workers=8) as executor:list(executor.map(one,manifest['files']))
    (directory/'asset_manifest.json').write_text(json.dumps(manifest,indent=2))
    print(f'Verified {len(manifest["files"])} pinned CC0 files: {directory}')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    fetch(parser.parse_args().directory)
