"""Download Qwen3-VL-4B-Instruct weights from ModelScope using multi-threaded aria2c."""
import os
import requests
import subprocess
from pathlib import Path

MODEL_ID = "Qwen/Qwen3-VL-4B-Instruct"
FILES = [
    "model-00001-of-00002.safetensors",
    "model-00002-of-00002.safetensors",
]

CACHE_DIR = Path(os.environ.get("MODELSCOPE_CACHE", "/workspace/qwen_robotnav/.modelscope/models")) / MODEL_ID
CACHE_DIR.mkdir(parents=True, exist_ok=True)

base_url = f"https://www.modelscope.cn/api/v1/models/{MODEL_ID}/repo?Revision=master&FilePath="

for fname in FILES:
    out_path = CACHE_DIR / fname
    if out_path.exists():
        print(f"{fname} already exists, skipping.")
        continue

    url = base_url + fname
    r = requests.get(url, allow_redirects=True, stream=True)
    final_url = r.url
    print(f"Downloading {fname} from {final_url[:80]}...")

    cmd = [
        "aria2c",
        "-x", "16",
        "-s", "16",
        "-k", "1M",
        "--max-connection-per-server=16",
        "--file-allocation=none",
        "-d", str(CACHE_DIR),
        "-o", fname,
        final_url,
    ]
    subprocess.run(cmd, check=True)

print("All weights downloaded to", CACHE_DIR)
