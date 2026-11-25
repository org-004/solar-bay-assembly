# Citation check C2 (S21-S40), run 2026-11-25

All 20 sources reachable on first attempt; none UNREACHABLE. Corrections are in citations-C2.json (9 entries).

## S21: OK
- Opened: raw GitHub URL at tag 4.0.0
- Checked: 1 claim group(s), 1 quotation(s); label Primary correct.
- Findings: Confirmation section present and marked optional; quote verbatim. Cited at 364, 374 (ok).

## S22: OK
- Opened: ozimmer.ch URL
- Checked: 2 claim group(s), 1 quotation(s); label Primary correct.
- Findings: Fairy Tale, Free Lunch Coupon, Dummy Alternative, Mega-ADR present; quote verbatim; delete-outdated allowed. Cited at 364-378 (ok: only pros = Fairy Tale, no consequences = Free Lunch, policy = Policy in Disguise).

## S23: CORRECTED
- Opened: media.pragprog.com PDF, text extracted from content streams
- Checked: 2 claim group(s), 1 quotation(s); label Primary correct.
- Findings: Quote verbatim (also 'DRY' Tip 15). Line 397 attaches a repository fact to S23: reworded to 'as DRY asks'.

## S24: OK
- Opened: docguide URL
- Checked: 3 claim group(s), 0 quotation(s); label Primary correct.
- Findings: Update with code, delete dead docs, link not duplicate, small fresh set: all present. Cited at 386, 398 (ok).

## S25: CORRECTED
- Opened: diataxis.fr/ and diataxis.fr/start-here/
- Checked: 2 claim group(s), 1 quotation(s); label Primary correct.
- Findings: Quote verbatim on start-here page. Line 387 says 'causes most documentation problems'; source says 'at the heart of a vast number of problems in documentation'.

## S26: OK
- Opened: archunit.org user guide
- Checked: 2 claim group(s), 1 quotation(s); label Primary correct.
- Findings: FreezingArchRule sentence verbatim; cycle checks, layer checks present. Cited at 405, 417 (ok).

## S27: OK
- Opened: qntm.org/ratchet
- Checked: 2 claim group(s), 1 quotation(s); label Primary correct.
- Findings: Quote verbatim; 'too many' raises error; numbers hard-coded. Cited at 407, 416, 422 (ok).

## S28: OK
- Opened: arxiv.org/abs/2602.11988 plus html v3
- Checked: 4 claim group(s), 1 quotation(s); label Primary correct.
- Findings: Quote verbatim (abstract); four agents (Claude Code, Codex x2 models, Qwen Code) and Python confirmed in full text; overviews not helpful. Cited at 429, 440, 444, 637 (ok).

## S29: OK
- Opened: code.claude.com/docs/en/memory
- Checked: 3 claim group(s), 1 quotation(s); label Primary correct.
- Findings: Quote verbatim; 'context, not enforced configuration'; 'target under 200 lines'. Cited 437, 443 (ok except the 'or CI' wording, see S30).

## S30: CORRECTED
- Opened: code.claude.com/docs/en/best-practices
- Checked: 3 claim group(s), 1 quotation(s); label Primary correct.
- Findings: Quote verbatim. Source says 'make it a hook, a deny rule, or a managed setting'; CI not named. Lines 140 and 437 changed.

## S31: CORRECTED
- Opened: arxiv.org/abs/2603.04177 plus html v2
- Checked: 2 claim group(s), 1 quotation(s); label Primary correct.
- Findings: Quote verbatim (prefix of abstract sentence). 100 tasks confirmed. Line 430 claim 'passing tests did not show the intended change was made' is not stated; reworded.

## S32: CORRECTED
- Opened: arxiv.org/abs/2602.15761 plus html
- Checked: 3 claim group(s), 1 quotation(s); label Primary correct.
- Findings: Quote verbatim; 19-35% and about 21% confirmed. Line 431 'single functions' wrong: datasets include program-level APPS.

## S33: OK
- Opened: GCC 14.2 Warning-Options
- Checked: 3 claim group(s), 1 quotation(s); label Primary correct.
- Findings: Quote verbatim; -Warray-bounds=2 false positives; -Wmaybe-uninitialized optimizing-only; listed flags not in -Wall/-Wextra. Cited 457, 472 (ok).

## S34: CORRECTED
- Opened: best.openssf.org guide
- Checked: 3 claim group(s), 1 quotation(s); label Primary correct.
- Findings: Quote verbatim. But OpenSSF recommends a blanket -Werror during development; line 459 'over a blanket one' reworded. Staged rollout with baseline confirmed. (Note: line 618 rejects blanket -Werror citing S34; S34 only objects for distributed source; left unchanged.)

## S35: OK
- Opened: clang.llvm.org DiagnosticsReference
- Checked: 3 claim group(s), 0 quotation(s); label Primary correct.
- Findings: -Wall/-Wmost/-Wextra/-Wconversion/-Wshadow-all present; -Wlogical-op absent (only -Wlogical-op-parentheses); -Wduplicated-cond absent. Cited 459, 475 (ok).

## S36: OK
- Opened: clang-tidy index
- Checked: 4 claim group(s), 0 quotation(s); label Primary correct.
- Findings: --warnings-as-errors 'does not enable any checks itself'; NOLINT(check); -p/compile_commands.json; RemovedArgs. Cited 482, 495 (ok).

## S37: CORRECTED
- Opened: bugprone-narrowing-conversions page
- Checked: 2 claim group(s), 0 quotation(s); label Primary correct.
- Findings: WarnWithinTemplateInstantiation default false confirmed; narrowing beyond -Wconversion confirmed. 'matters for Eigen code' not on the page (author inference): marked as such in entry.

## S38: CORRECTED
- Opened: GCC 14.2 Static-Analyzer-Options; gcc-13/14/15 changes.html
- Checked: 1 claim group(s), 1 quotation(s); label Primary correct.
- Findings: Quote is NOT on the options page; it is verbatim in release notes of GCC 13, 14 and 15. Entry gives only the options URL: release-notes URL added. Cited 489, 499, 623 (ok).

## S39: OK
- Opened: GCC 14.2 Instrumentation-Options
- Checked: 3 claim group(s), 1 quotation(s); label Primary correct.
- Findings: Quote verbatim; ASan cannot combine with TSan; float-divide-by-zero not in undefined. Cited 490, 499, 625 (ok).

## S40: OK
- Opened: clang.llvm.org MemorySanitizer
- Checked: 1 claim group(s), 1 quotation(s); label Primary correct.
- Findings: Quote verbatim; 'even libc' instrumentation. Cited 490, 625 (ok; 'needs Clang' is an inference from the page being Clang-only).
