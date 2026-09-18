"""测试套件自身的完整性检查。

存在理由：2026-09-18 我在同一个文件上分几次追加测试时，
**把整块内容追加了两遍** —— 于是同名测试定义出现两次，
而 Python 里**后定义的会静默覆盖前面的**。表现出来是：

* 我明明改好了某个测试的断言，它却仍然按旧断言失败（或反过来通过）
* 排查时看到的是「测试和代码对不上」，而真正的原因是文件里有两份

这和本项目一路在猎杀的那类缺陷是同一个形状：**不报错、结果不对、难以归因**。
"""

from __future__ import annotations

import ast
import collections
import glob
import os


def test_no_duplicate_top_level_definitions():
    """同一个测试文件里不允许出现重名的顶层函数。

    重名时后定义的会静默覆盖前面的 —— 你以为在跑 v2，实际跑的是 v1。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    problems = []
    for path in sorted(glob.glob(os.path.join(here, "test_*.py"))):
        tree = ast.parse(open(path, encoding="utf-8").read())
        names = [n.name for n in tree.body if isinstance(n, ast.FunctionDef)]
        dups = sorted(k for k, v in collections.Counter(names).items() if v > 1)
        if dups:
            problems.append(f"{os.path.basename(path)}: {dups}")

    assert not problems, (
        "以下文件有重名的顶层定义（后定义的会静默覆盖前面的）：\n  "
        + "\n  ".join(problems)
    )
