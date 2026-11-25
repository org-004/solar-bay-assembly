#!/usr/bin/env python3
"""compdb.py [--config NAME]... [--for-clang] OUTDIR

Writes OUTDIR/compile_commands.json: one entry for every translation unit of
this repository (each .cc under src/ and tests/), with the command Bazel
would compile it with under --config=opt and the configs given. The commands
come from `bazel aquery`, so the flags of .bazelrc (--per_file_copt among
them) are in them and nothing is listed twice by hand. A file that is
compiled into several targets is entered once, with its first command.

The entries' directory is Bazel's execution root, where the headers of the
outside libraries are found once the repository has been built
(`bazel build --config=opt //...`). OUTDIR is the caller's; nothing is
written into the checkout.

--for-clang drops what only GCC understands (the -W options and
-fno-canonical-system-headers), for clang-tidy, which reads the database but
is another compiler. The environment variable BAZEL, if set, is the Bazel
command to run, startup options included (default: bazel).

Used by tools/architecture/warnings.sh and tools/architecture/tidy.sh.
"""
import json
import os
import shlex
import subprocess
import sys


def main(argv):
    configs, for_clang, outdir = [], False, None
    i = 1
    while i < len(argv):
        a = argv[i]
        if a == "--config":
            i += 1
            configs.append(argv[i])
        elif a == "--for-clang":
            for_clang = True
        elif a.startswith("-"):
            sys.exit(__doc__)
        else:
            outdir = a
        i += 1
    if outdir is None:
        sys.exit(__doc__)
    bazel = shlex.split(os.environ.get("BAZEL", "bazel"))
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    flags = ["--config=opt"] + ["--config=" + c for c in configs]
    execroot = subprocess.run(
        bazel + ["info"] + flags + ["execution_root"], cwd=root, check=True,
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True).stdout.strip()
    out = subprocess.run(
        bazel + ["aquery"] + flags + ['mnemonic("CppCompile", //...)', "--output=jsonproto"],
        cwd=root, check=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True).stdout
    entries, seen = [], set()
    for action in json.loads(out).get("actions", []):
        args = action["arguments"]
        files = [x for x in args if x.startswith(("src/", "tests/")) and x.endswith(".cc")]
        if not files or files[0] in seen:
            continue
        seen.add(files[0])
        if for_clang:
            args = [x for x in args
                    if not x.startswith("-W") and x != "-fno-canonical-system-headers"]
        entries.append({"directory": execroot, "file": files[0], "arguments": args})
    entries.sort(key=lambda e: e["file"])
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "compile_commands.json"), "w") as f:
        json.dump(entries, f, indent=1)
    print("%d translation units -> %s" % (len(entries), os.path.join(outdir, "compile_commands.json")))


if __name__ == "__main__":
    main(sys.argv)
