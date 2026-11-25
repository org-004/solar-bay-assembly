# Machine state left by planning (run 013)

- Repo clone: /root/work/013-design-principles-and-r1/repo, branch org/013-design-principles-and at B=940a01b, pushed.
- Bazel: ~/.local/bin/bazel (add to PATH). Remote cache via /etc/bazel.bazelrc. No disk cache (removed for disk).
- Docker: /etc/docker/daemon.json makes /usr/local/bin/runc-nokey the default runtime (kernel key quota).
  `docker build` and `docker commit` do not work. Long-lived container `sba-ci` (ubuntu:24.04, g++ 13.3,
  clang-tidy-18 18.1.3, python3, git) mounts /root/work and /root/.cache; use
  `docker exec -w <repo> sba-ci bazel --output_base=/root/.cache/bazel/ob-ubuntu ...`.
- Host has clang-tidy-18 (18.1.8). No /usr/bin/time; read /sys/fs/cgroup/memory.peak.
- Probe logs: logs/strict1.log (68 candidate flags), logs/probe_programs.log, logs/progs/, logs/tidy_*.log,
  logs/aquery.json, logs/compile_commands.json.
- Token cannot push .github/workflows/* (refused). Disk: about 0.4 GB free; check `df -h /` often.
- `//:ompl` is an alias of @ompl//:ompl: leave it out of `--output_groups=compilation_outputs //...`.
