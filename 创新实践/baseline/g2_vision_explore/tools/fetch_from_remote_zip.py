#!/usr/bin/env python3
"""从远端 zip 里按需取几个文件，不下载整个包。

--------------------------------------------------------------------------------
为什么需要这个

公开数据集常打包成整个几 GB 的 zip，但验证一个假设往往只需要几十张图。
这台共享服务器磁盘已占用 95%，为了几张图拉两个 G 不合适。

zip 的结构允许这么做：中央目录（文件清单）在**文件末尾**，
每个条目在本地文件头之后就是压缩数据。所以只要用 HTTP Range 请求：

    1. 取末尾若干字节，解析出中央目录 -> 得到每个条目的 offset/size
    2. 对想要的条目，Range 请求它那段字节 -> 解压

用法::

    # 先列出包里有什么
    python3 tools/fetch_from_remote_zip.py URL --list

    # 取前 20 张图到目录
    python3 tools/fetch_from_remote_zip.py URL --out DIR --pattern '.*\\.(jpg|png)$' --limit 20

只依赖标准库。
"""

from __future__ import annotations

import argparse
import io
import re
import struct
import sys
import urllib.request
import zipfile
from pathlib import Path

CENTRAL_DIR_SIG = b"PK\x01\x02"
EOCD_SIG = b"PK\x05\x06"


class HttpRangeFile(io.RawIOBase):
    """把一个 HTTP 资源伪装成可随机 seek 的文件对象，供 zipfile 使用。"""

    def __init__(self, url: str, block_size: int = 1 << 16):
        self.url = url
        self.block_size = block_size
        self.pos = 0
        req = urllib.request.Request(url, method="HEAD")
        with urllib.request.urlopen(req, timeout=60) as resp:
            self.size = int(resp.headers["Content-Length"])

    def seek(self, offset: int, whence: int = io.SEEK_SET) -> int:
        if whence == io.SEEK_SET:
            self.pos = offset
        elif whence == io.SEEK_CUR:
            self.pos += offset
        elif whence == io.SEEK_END:
            self.pos = self.size + offset
        return self.pos

    def tell(self) -> int:
        return self.pos

    def seekable(self) -> bool:
        return True

    def readable(self) -> bool:
        return True

    def read(self, n: int = -1) -> bytes:  # noqa: D102
        if n is None or n < 0:
            n = self.size - self.pos
        if n == 0:
            return b""
        end = min(self.pos + n, self.size) - 1
        if end < self.pos:
            return b""
        req = urllib.request.Request(
            self.url, headers={"Range": f"bytes={self.pos}-{end}"}
        )
        with urllib.request.urlopen(req, timeout=120) as resp:
            # 必须校验服务端真的支持 Range。若某个镜像忽略 Range 头、
            # 直接返回 200 + 整个文件，那么拿到的 data 是**从 0 开始**的内容，
            # 却被当成 self.pos 处的数据 —— 会静默产出错位的数据。
            # （zipfile 的 CRC 能兜住一部分，但那是"响亮失败"，不如这里直接挡。）
            if resp.status != 206:
                raise RuntimeError(
                    f"服务端未按 Range 返回（HTTP {resp.status}）—— "
                    f"该源不支持分段读取，本工具的结果不可信"
                )
            data = resp.read()
        self.pos += len(data)
        return data

    def readall(self) -> bytes:  # noqa: D102
        return self.read(-1)


def main() -> int:
    ap = argparse.ArgumentParser(description="按需从远端 zip 取文件")
    ap.add_argument("url")
    ap.add_argument("--list", action="store_true", help="只列出内容")
    ap.add_argument("--out", default=None, help="输出目录")
    ap.add_argument("--pattern", default=r".*\.(jpg|jpeg|png)$", help="文件名正则")
    ap.add_argument("--limit", type=int, default=20, help="最多取几个")
    ap.add_argument("--stride", type=int, default=1,
                    help="每隔几个匹配项取一个。数据集常是连续录像帧，"
                         "相邻帧几乎一样；设成大一点的值能跨时间采样，"
                         "覆盖到不同的姿态与事件")
    args = ap.parse_args()

    print(f"连接 {args.url}")
    rf = HttpRangeFile(args.url)
    print(f"  远端大小 {rf.size / 1048576:.1f} MB（只会传输实际需要的部分）")

    zf = zipfile.ZipFile(rf)
    names = zf.namelist()
    print(f"  包内条目 {len(names)} 个")

    if args.list:
        for n in names[:60]:
            print("   ", n)
        if len(names) > 60:
            print(f"    ...（共 {len(names)}）")
        return 0

    if not args.out:
        print("需要 --out 或 --list")
        return 1

    pat = re.compile(args.pattern, re.I)
    matched = [n for n in names if pat.search(n) and not n.endswith("/")]
    stride = max(1, args.stride)
    targets = matched[::stride][: args.limit]
    if not targets:
        print("没有条目匹配该模式")
        return 1
    print(f"  匹配 {len(matched)} 项，按 stride={stride} 采样出 {len(targets)} 项")

    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)

    # ⚠️ 这里必须数**成功落盘**的，不能数尝试数。
    #
    # 踩过的坑：第一版打印 `len(targets)`（尝试数）。实跑时请求 190 个、
    # 磁盘上只有 138 个 —— 尾部 47 个静默没下成，而汇总行仍然报"共取 190 个"。
    # 结果是一份评估报告建立在一个缺了 26% 录像（含 2 个倒地事件）的子集上，
    # 而所有人都以为数据是完整的。
    #
    # 静默少给数据是最危险的一类 bug：程序不崩，只是结论错。
    ok = 0
    failed: list[str] = []
    total = 0
    for n in targets:
        dest = outdir / Path(n).name
        try:
            data = zf.read(n)
            dest.write_bytes(data)
        except Exception as exc:
            failed.append(f"{Path(n).name}: {exc}")
            dest.unlink(missing_ok=True)   # 不留残文件
            continue

        # 写后校验：写盘环节可能静默失败（磁盘满、中断），产生 0 字节文件。
        if not dest.exists() or dest.stat().st_size != len(data):
            got = dest.stat().st_size if dest.exists() else -1
            failed.append(f"{Path(n).name}: 落盘大小不符（期望 {len(data)}，实得 {got}）")
            dest.unlink(missing_ok=True)
            continue

        ok += 1
        total += len(data)

    print(f"\n成功取回 {ok}/{len(targets)} 个文件，{total / 1048576:.1f} MB -> {outdir}")
    if failed:
        print(f"⚠️ **{len(failed)} 个失败**（这份数据是不完整的，别直接拿去评估）：")
        for f in failed[:10]:
            print(f"    ✗ {f}")
        if len(failed) > 10:
            print(f"    ...（共 {len(failed)} 个）")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
