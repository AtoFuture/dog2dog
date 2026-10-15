"""Download Qwen3-VL-4B-Instruct weights using aria2c multi-connection download."""

import os
import sys
import requests
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed

MODEL_ID = "Qwen/Qwen3-VL-4B-Instruct"
FILES = [
    "model-00001-of-00002.safetensors",
    "model-00002-of-00002.safetensors",
]

CACHE_DIR = Path(os.environ.get("MODELSCOPE_CACHE", "/workspace/qwen_robotnav/.modelscope")) / MODEL_ID
CACHE_DIR.mkdir(parents=True, exist_ok=True)

BASE_URL = f"https://www.modelscope.cn/api/v1/models/{MODEL_ID}/repo?Revision=master&FilePath="


def get_final_url(fname: str) -> str:
    url = BASE_URL + fname
    with requests.get(url, allow_redirects=True, stream=True, timeout=60) as resp:
        resp.raise_for_status()
        return resp.url


def download_file(fname: str) -> None:
    out_path = CACHE_DIR / fname
    final_url = get_final_url(fname)
    print(f"Downloading {fname} via aria2c ...")
    print(f"URL: {final_url[:80]}...")

    cmd = [
        "aria2c",
        "-x", "16",
        "-s", "16",
        "-k", "1M",
        "--max-connection-per-server=16",
        "--min-split-size=1M",
        "--file-allocation=none",
        "--continue=true",
        "-d", str(CACHE_DIR),
        "-o", fname,
        final_url,
    ]
    import subprocess
    subprocess.run(cmd, check=True)
    print(f"{fname} done.")


def main():
    for fname in FILES:
        try:
            download_file(fname)
        except Exception as exc:
            print(f"ERROR downloading {fname}: {exc}", file=sys.stderr)
            sys.exit(1)
    print("All weights downloaded to", CACHE_DIR)


if __name__ == "__main__":
    main()
