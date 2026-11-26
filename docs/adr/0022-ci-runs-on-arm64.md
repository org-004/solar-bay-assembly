# 0022 CI runs on 64-bit ARM, the platform the code is developed and proved on

Status: accepted
Date: 2026-11-25
Decided by: the owner ("CI should be arm64. No need to complicate things."), commit
`ca208e1`

**Context.** Every engineer's machine is 64-bit ARM, and until `ca208e1` CI ran on
GitHub's `ubuntu-latest`, which is x86-64. The commit before it, `e4e4710`, made 29
compiler warnings errors for `src/` and `tests/` (ADR 0014). They were proved clean
under GCC 14.2 and GCC 13.3 on ARM and by a syntax-only parse with a cross compiler for
x86-64, because no engineer's machine can build for x86-64. On `e4e4710` the six test
jobs of CI failed within two minutes in "Build and test"; on `940a01b`, the commit
before, they had passed. Two platforms meant that what an engineer proved was not what
CI checked, and that a rule could only be half proved before it landed.

**Decision.** Every job of `.github/workflows/ci.yml` runs on GitHub's hosted arm64
runner for Ubuntu 24.04 (`ubuntu-24.04-arm`). The disk caches carry `arm64` in their
names, so that nothing built for x86-64 is restored. The repository is built and
tested on one platform, 64-bit ARM Linux.

**Alternatives.** Keep CI on x86-64 and move the flags that fail there to
`--config=strict`: lost, because it keeps a platform that no development machine can
build for and a list of exceptions that grows with it. Both platforms in CI: lost,
because it doubles CI's time for a platform nobody runs the cell on.

**Consequences.** What passes in an engineer's container is what CI checks: the same
compiler family, the same counts for the warnings and for `clang-tidy`. x86-64 is now
built and tested nowhere; a difference there would be found only by someone who builds
there, and this record is where they learn that nobody did. The Context of the records
0002, 0006, 0014 and 0021 names x86-64 as CI's platform; that was so when they were
written and is not so now. Their decisions stand. The rules of ADR 0006 stay in force
with a narrower reason: results must not depend on the machine or on the library's
version, though the platform is one.

**Check.** `grep -c "runs-on: ubuntu-24.04-arm" .github/workflows/ci.yml` prints the
number of jobs, and `grep -c "ubuntu-latest" .github/workflows/ci.yml` prints 0.

**Migration.** None: the workflow already follows this.
