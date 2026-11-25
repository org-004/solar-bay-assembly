# The Ubuntu 24.04 container (sba-ci), commands that worked

Image `sba-ci:1` (Ubuntu 24.04, g++ 13.3.0, clang-tidy-18 18.1.3, Bazelisk). The container is left running.

1. Start (once):
   docker run -d --name sba-ci -v /root/work:/root/work -v /root/.cache:/root/.cache sba-ci:1 sleep infinity

2. Bazel inside the container: `bazel --output_base=/root/.cache/bazel/ob-ubuntu ...` (own server, own output base; the
   repository's .bazelversion makes Bazelisk fetch 9.3.0). `--symlink_prefix` is a command option, not a startup option,
   so it cannot go in the BAZEL variable; `bazel query/info/aquery` (all the tools use) create no convenience links.
   Only the `build` below got `--symlink_prefix=/`, which makes Bazel write the links to `/bazel-*` (inside the container
   only, nowhere in the checkout). Afterwards the checkout's bazel-bin, bazel-out, bazel-repo and bazel-testlogs still
   pointed into the host's output base (checked with `ls -la repo | grep bazel-`).

3. Make the outside headers present in the container's output base (compiles every own file, no MuJoCo; 150 s
   including a wait for the lock on 2026-11-25; the first line only lists the targets):
   T=$(docker exec -w /root/work/013-design-principles-and-r2/repo sba-ci bazel --output_base=/root/.cache/bazel/ob-ubuntu query 'kind("cc_.*", //...)' | tr '\n' ' ')
   flock -o /root/work/013-design-principles-and-r2/heavy.lock docker exec -w /root/work/013-design-principles-and-r2/repo sba-ci \
     bazel --output_base=/root/.cache/bazel/ob-ubuntu build --config=opt --symlink_prefix=/ --jobs=3 --local_resources=cpu=3 \
     --output_groups=compilation_outputs $T
   (the same as out/counts/docker/build.sh, which was run with `flock` without `-o`)

4. The tools, in the container (the same arguments as on the host, one more option set):
   docker exec -w /root/work/013-design-principles-and-r2/repo -e "BAZEL=bazel --output_base=/root/.cache/bazel/ob-ubuntu" sba-ci \
     tools/architecture/warnings.sh --flags FLAG,FLAG,... --jobs 3 --raw ABSOLUTE_DIR
   docker exec -w /root/work/013-design-principles-and-r2/repo -e CLANG_TIDY=clang-tidy-18 \
     -e "BAZEL=bazel --output_base=/root/.cache/bazel/ob-ubuntu" sba-ci \
     tools/architecture/tidy.sh --checks CHECKS --jobs 3 --raw ABSOLUTE_DIR
   Both under `flock -o <lock>` on the host side. `--raw` needs an absolute path: a relative one is taken relative to the
   checkout (the container's working directory) and writes into it (this happened once and was moved out at once).
