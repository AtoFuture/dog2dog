"""Multi-threaded download of Qwen3-VL-4B-Instruct weights from ModelScope.

Each chunk is saved to a separate .part file and merged only when all parts
succeed. The ModelScope URL is re-resolved for each chunk so the auth_key is
always fresh.
"""

import os
import sys
import math
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
CHUNK_SIZE = 128 * 1024 * 1024  # 64 MB per thread
MAX_WORKERS = 12


def resolve_url(fname: str) -> tuple:
    """Return (modelscope_url, total_size, supports_range)."""
    url = BASE_URL + fname
    with requests.get(url, allow_redirects=True, stream=True, timeout=60) as resp:
        resp.raise_for_status()
        total = int(resp.headers.get("content-length", 0))
        supports_range = resp.headers.get("accept-ranges") == "bytes"
        return url, total, supports_range


def download_part(modelscope_url: str, start: int, end: int, part_path: Path, part_idx: int) -> int:
    """Download a single [start, end] range to part_path."""
    if part_path.exists():
        existing = part_path.stat().st_size
        expected = end - start + 1
        if existing == expected:
            print(f"  part {part_idx}: already complete ({existing / 1024 / 1024:.1f} MB)")
            return existing

    headers = {"Range": f"bytes={start}-{end}"}
    downloaded = 0
    with requests.get(modelscope_url, headers=headers, stream=True, timeout=600) as resp:
        resp.raise_for_status()
        with open(part_path, "wb") as f:
            for chunk in resp.iter_content(chunk_size=1024 * 1024):
                if chunk:
                    f.write(chunk)
                    downloaded += len(chunk)
    print(f"  part {part_idx}: {start}-{end} done ({downloaded / 1024 / 1024:.1f} MB)")
    return downloaded


def merge_parts(out_path: Path, num_parts: int) -> None:
    with open(out_path, "wb") as out_f:
        for i in range(num_parts):
            part_path = Path(f"{out_path}.part{i}")
            with open(part_path, "rb") as part_f:
                while True:
                    data = part_f.read(8 * 1024 * 1024)
                    if not data:
                        break
                    out_f.write(data)
            part_path.unlink()


def download_file(fname: str) -> None:
    out_path = CACHE_DIR / fname
    print(f"Resolving {fname} ...")
    modelscope_url, total, supports_range = resolve_url(fname)
    print(f"{fname}: {total / 1024 / 1024 / 1024:.2f} GB, range={supports_range}")

    if not supports_range:
        # Fall back to single-threaded streaming download.
        print(f"Range not supported, downloading {fname} single-threaded ...")
        with requests.get(modelscope_url, stream=True, timeout=600) as resp, open(out_path, "wb") as f:
            resp.raise_for_status()
            for chunk in resp.iter_content(chunk_size=1024 * 1024):
                if chunk:
                    f.write(chunk)
        print(f"{fname} downloaded.")
        return

    num_parts = math.ceil(total / CHUNK_SIZE)
    workers = min(MAX_WORKERS, num_parts)

    futures = []
    with ThreadPoolExecutor(max_workers=workers) as executor:
        for i in range(num_parts):
            start = i * CHUNK_SIZE
            end = min((i + 1) * CHUNK_SIZE - 1, total - 1)
            part_path = Path(f"{out_path}.part{i}")
            futures.append(executor.submit(download_part, modelscope_url, start, end, part_path, i))

        for future in as_completed(futures):
            future.result()

    print(f"Merging {num_parts} parts for {fname} ...")
    merge_parts(out_path, num_parts)

    actual = out_path.stat().st_size
    if actual != total:
        raise RuntimeError(f"{fname} size mismatch: {actual} != {total}")
    print(f"{fname} complete ({actual / 1024 / 1024 / 1024:.2f} GB).")


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
