# Research reader E: decision records, one home per fact, repositories for coding agents

All pages fetched 2026-11-25 with status 200. 24 source blocks, all primary.

## (e) Decision records

### Nygard, Documenting Architecture Decisions
- Citation: Michael Nygard, "Documenting Architecture Decisions", 2011-11-15. https://www.cognitect.com/blog/2011/11/15/documenting-architecture-decisions
- Kind: primary
- Read: 2026-11-25, whole
- Topics: e
- Claim 1: The original form has five parts: Title (short noun phrase), Context (forces, value-neutral), Decision (full sentences, "We will ..."), Status (proposed, accepted, deprecated or superseded with a reference to the replacement), Consequences (all of them, positive, negative and neutral). One ADR holds one significant decision; the whole record is one or two pages.
  Quote: "ADRs will be numbered sequentially and monotonically. Numbers will not be reused."
- Claim 2: A reversed decision is kept and marked superseded, because it is still relevant to know what was decided. Scope is "architecturally significant" decisions: those affecting structure, non-functional characteristics, dependencies, interfaces or construction techniques. Records live in the repository (his path: doc/arch/adr-NNN.md) in lightweight markup.
- Claim 3: His reason for small records: large documents are not kept up to date and nobody reads them. His evidence is an experience report of a few months (six to ten developers rotating through projects) with positive feedback, not a measurement.
  Quote: "Large documents are never kept up to date. Small, modular documents have at least a chance at being updated."
- Bears on this repository: A rule "each decision record has Title, Status, Context, Decision, Consequences, one decision per file, numbers never reused" can be checked by a script over `doc/adr/*.md`.

### Fowler, Architecture Decision Record (bliki)
- Citation: Martin Fowler, "Architecture Decision Record", dated 24 March 2026. https://martinfowler.com/bliki/ArchitectureDecisionRecord.html
- Kind: primary (Fowler's own note; it summarizes Nygard but adds his own lifecycle wording)
- Read: 2026-11-25, whole
- Topics: e
- Claim 1: Lifecycle is proposed, accepted, superseded. An accepted record is never reopened or edited; it is superseded by a new record, and the old one links to the new one. The log therefore shows how long each decision governed the work.
  Quote: "Once an ADR is accepted, it should never be reopened or changed - instead it should be superseded."
- Claim 2: Each record is its own file, numbered in a monotonic sequence in the file name, with a name that captures the decision (his example "0001-HTMX-for-active-web-pages"); stored in the code repository, commonly doc/adr, in markdown. Brevity is the most important thing: typically a single page, link supporting material. Use inverted-pyramid order. List serious alternatives with pros and cons, state consequences, and record the confidence level and what changes should trigger re-evaluation.
- Claim 3: Writing the record is valued as much as reading it, since it forces differing views into the open.
- Bears on this repository: Checkable rules: file name matches `NNNN-short-title.md`; status field is one of three values; a record whose status is "superseded" names its successor and the successor names it; accepted records are not edited (diff check in CI against base branch, except the status line).

### MADR 4.0.0 template
- Citation: ADR organization, "MADR (Markdown Any Decision Records) adr-template.md", tag 4.0.0. https://raw.githubusercontent.com/adr/madr/4.0.0/template/adr-template.md
- Kind: primary (the template file itself)
- Read: 2026-11-25, whole
- Topics: e
- Claim 1: Structure is YAML front matter (status, date, decision-makers, consulted, informed; all marked optional), then Context and Problem Statement, Decision Drivers (optional), Considered Options, Decision Outcome ("Chosen option: ..., because ..."), Consequences (Good/Bad, optional), Confirmation (optional), Pros and Cons of the Options (optional), More Information (optional). The status values listed are proposed, rejected, accepted, deprecated, superseded by ADR-0123.
- Claim 2: The "Confirmation" section asks how compliance with the decision can be verified, with a design review or an architecture test as examples. This is the template's hook for a checkable rule.
  Quote: "Describe how the implementation of/compliance with the ADR can/will be confirmed."
- Bears on this repository: A trimmed MADR (status, context, options, outcome, consequences, confirmation) fits; Confirmation can name the command that checks the decision (a Bazel query, a test, a lint). The consulted/informed fields do not fit a one-owner repository.

### adr.github.io overview
- Citation: ADR GitHub organization, "Architectural Decision Records" home page, 2026. https://adr.github.io/
- Kind: primary (the community site's own text; it is a link hub, so thin on content)
- Read: 2026-11-25, in part (landing page text: pointers to Nygard's post, Zimmermann's history "Architectural Decisions - The Making Of", a seven-template comparison paper)
- Topics: e
- Claim 1: The site names Nygard's 2011 post as the one that popularized the form and points to Zimmermann's history of decision recording since the late 1990s and a WICSA 2015 comparison of seven templates; it is an index of templates, tools and practices, not itself a specification.
- Bears on this repository: None as a rule; used only to confirm that MADR and Nygard are the two mainstream templates to choose between.

### Zimmermann, How to create ADRs and how not to
- Citation: Olaf Zimmermann, "How to create Architectural Decision Records (ADRs) — and how not to", 2023-04-03 (page updated Sep 2026). https://ozimmer.ch/practices/2023/04/03/ADRCreation.html
- Kind: primary (author's own text; practitioner experience, no measurement)
- Read: 2026-11-25, whole
- Topics: e
- Claim 1: He lists named anti-patterns: Fairy Tale (only pros, tautologies), Sales Pitch, Free Lunch Coupon (no or only harmless consequences), Dummy Alternative (invented option), Sprint (one option, short-term effects only), Tunnel Vision, Maze (content off-topic), Blueprint or Policy in Disguise (cookbook or law tone), Mega-ADR (detailed architecture stuffed into multi-page ADRs; equivalent to a "god class"; remedy: move the detail design to a separate document), Novel/epic, and "magic tricks" (false urgency, solution looking for a problem, pseudo-accuracy of weighted scores).
  Quote: "Mega-ADR: A lot of detailed information about the architecture is stuffed into several multi-page ADRs serving as documentation master"
- Claim 2: Good practice: select by architectural significance; a record logs the resolution of at least one significant requirement; watch word count (a few sentences to a few pages); disclose confidence; list at least two real options (a footnote says fake alternatives just to fit the template are not wanted: sometimes there is one choice, which should still be justified). He says he did not cover ADR maintenance and that outdated records may be marked superseded or even deleted.
- Bears on this repository: Reviewable yes-or-no rules for an ADR: states at least one consequence that is negative; length under a line limit; no component specifications or code beyond a snippet per option; options are real. Records restating what the code already shows fall under Mega-ADR/Blueprint.

### Zimmermann, Definition of Done for architectural decisions
- Citation: Olaf Zimmermann, "A Definition of Done for Architectural Decision Making", 2020-05-22 (updated Aug 2026). https://www.ozimmer.ch/practices/2020/05/22/ADDefinitionOfDone.html
- Kind: primary (author's proposal; he says he has not yet applied the checklist as such in practice)
- Read: 2026-11-25, whole
- Topics: e
- Claim 1: Five criteria, "ecADR": Evidence (reason to believe it works, does not break earlier ADs, is actionable), Criteria (at least two alternatives compared), Agreement (a peer or the team challenged it), Documentation (lean template such as a Y-statement or MADR), Realization and review plan (scheduled, when to check it was implemented as intended, when to revisit).
- Claim 2: A five-question yes-or-no checklist ends the post; if any answer is missing, the decision is not done. Review plan item (b) says architecture tests (he names ArchUnit) are part of evaluating whether the decision was implemented.
  Quote: "If you can answer “yes” five times in this quick test, you are done with an AD."
- Bears on this repository: Gives a closing checklist for an ADR written by an agent; "Agreement" must be read as the owner's acceptance, and "Realization" as the check command plus a re-visit trigger.

### AWS Prescriptive Guidance, ADR process
- Citation: Amazon Web Services, "Architectural decision record process", AWS Prescriptive Guidance (undated). https://docs.aws.amazon.com/prescriptive-guidance/latest/architectural-decision-records/adr-process.html
- Kind: primary (a vendor's own guidance document about a process it recommends; no data)
- Read: 2026-11-25, whole
- Topics: e
- Claim 1: States: proposed, then accepted or rejected (with a reason recorded to prevent the same debate); accepted and rejected records are immutable; a change means a new ADR, and on approval the old one is set to Superseded. Minimum contents are context, decision, consequences.
  Quote: "When the team accepts an ADR, it becomes immutable."
- Claim 2: ADRs are used as a reference in code review: a reviewer who finds a change violating an ADR asks the author to update the code and links the ADR.
- Bears on this repository: Supports a rule "a pull request that contradicts an accepted ADR is rejected or comes with a superseding ADR", and recording rejected options as rejected records.

## (f) One home per fact, documentation upkeep

### Pragmatic Programmer, DRY chapter excerpt
- Citation: Andrew Hunt and David Thomas, "The Pragmatic Programmer, 20th Anniversary Edition", Topic 9 "DRY", publisher's sample (PDF). https://media.pragprog.com/titles/tpp20/dry.pdf
- Kind: primary
- Read: 2026-11-25, in part (the sample extract; I decompressed the PDF text myself, so only passages that came out intact are used)
- Topics: f
- Claim 1: DRY is stated about knowledge, not only code lines. The comment-and-code example shows the failure: the intent is given twice, so a change must be made twice, and in time comment and code diverge.
  Quote: "Every piece of knowledge must have a single, unambiguous, authoritative representation within a system."
- Claim 2: For interfaces they advise a neutral specification from which tools generate documentation, mocks, tests and clients, ideally kept in a central repository; that is the "generated reference" pattern.
- Bears on this repository: Rules of the kind "a number, a command, a target list or a file layout appears in one file; other documents link to it or are generated from it" can be checked by grep for duplicated literals (e.g. the test command in AGENTS.md and README).

### Google documentation best practices (docguide)
- Citation: Google, "Documentation Best Practices", google/styleguide docguide. https://google.github.io/styleguide/docguide/best_practices.html
- Kind: primary
- Read: 2026-11-25, whole
- Topics: f
- Claim 1: Headings: Minimum Viable Documentation, Update Docs with Code (same change list as the code), Delete Dead Documentation, Prefer the Good Over the Perfect, Documentation is the Story of Your Code, Duplication is Evil. Dead docs misinform and slow people; default to delete, stragglers can be recovered.
  Quote: "A small set of fresh and accurate docs is better than a large assembly of “documentation” in various states of disrepair."
- Claim 2: Design docs, once implemented, should serve as archives of decisions, not as half-correct docs. Behavior documented in method API docs should have a test verifying it; "why" belongs in inline comments, not API docs. Duplication: link to the existing guide instead of writing your own.
- Bears on this repository: Yes-or-no review rules: a change that alters a command or behavior also changes the doc naming it; a documented behavior has a test; a doc describing a replaced "stand-in cell" is deleted or marked obsolete when the cell is replaced.

### Software Engineering at Google, chapter 10 Documentation
- Citation: Titus Winters, Tom Manshreck, Hyrum Wright (eds.), "Software Engineering at Google", chapter 10 "Documentation", 2020. https://abseil.io/resources/swe-book/html/ch10.html
- Kind: primary
- Read: 2026-11-25, in part (sections "Documentation Is Like Code", "Know Your Audience" passages, "Deprecating Documents"; searched the rest for the relevant terms)
- Topics: f
- Claim 1: Documents should be treated as code: owners, source control beside the code, review. When owners conflict, designate canonical documentation and consolidate or deprecate duplicates; placing documents under source control next to the code helps make them canonical.
  Quote: "determine the primary source and consolidate other associated documents into that primary source (or deprecate the duplicates)"
- Claim 2: Stale documents are removed or marked obsolete with a pointer to new information. Google attaches "freshness dates" with an owner and last-reviewed date, with email reminders after a period (example: three months); including the owner in a visible byline raised adoption. Design decisions do not belong in API documents; put them in a design doc.
- Bears on this repository: A "last reviewed" front-matter field is checkable but needs a reminder mechanism; with one owner and agents that is likely overhead (see Does not fit).

### Diataxis
- Citation: Daniele Procida, "Diátaxis: a systematic approach to technical documentation authoring" and "Start here - Diátaxis in five minutes". https://diataxis.fr/ and https://diataxis.fr/start-here/
- Kind: primary
- Read: 2026-11-25, whole of both pages
- Topics: f
- Claim 1: Four kinds of documentation answering four needs: tutorials (learning, practical), how-to guides (a goal for a competent user), reference (neutral facts, structure mirrors the thing described), explanation (why, context). Mixing them is claimed to be the root of many documentation problems; tutorials should not carry long explanation, link to it.
  Quote: "Crossing or blurring the boundaries described in the map is at the heart of a vast number of problems in documentation."
- Claim 2: The site's evidence is adoption testimonials (Vonage, Gatsby, Cloudflare), not a measurement. It says to apply it to something small rather than read everything first.
- Bears on this repository: Two of the four kinds map onto real needs here, reference (target list, CSV columns, layout JSON schema) and explanation (why, i.e. ADRs); a rule "each document states which one kind it is and does not mix" is checkable by a header. Tutorials and how-to quadrants largely do not (see Does not fit).

## (h) Repositories for coding agents

### AGENTS.md site
- Citation: AGENTS.md project (stewarded by the Agentic AI Foundation, Linux Foundation), "AGENTS.md: a simple, open format for guiding coding agents". https://agents.md/
- Kind: primary
- Read: 2026-11-25, whole
- Topics: h
- Claim 1: It is plain Markdown with no required fields; described as a "README for agents": build steps, tests and conventions that would clutter the human README. Suggested sections: project overview, build and test commands, code style, testing instructions, security. For conflicts, the nearest file to the edited file wins and explicit user prompts override everything. Nested files are for large monorepos.
  Quote: "The closest AGENTS.md to the edited file wins; explicit user chat prompts override everything."
- Claim 2: It says the agent will run test commands listed in the file and try to fix failures. This is the project's own claim; no measurement is given on the page.
- Bears on this repository: One root AGENTS.md with exact build, test, gate commands. Nested files are not needed for a flat tree (see Does not fit).

### Claude Code, memory (CLAUDE.md) documentation
- Citation: Anthropic, "How Claude remembers your project" (Claude Code docs, memory page). https://code.claude.com/docs/en/memory
- Kind: primary (vendor documenting its own product; recommendations, no data)
- Read: 2026-11-25, in part (sections on writing instructions, troubleshooting, size limits, AGENTS.md)
- Topics: h
- Claim 1: CLAUDE.md is context delivered as a user message, not enforced configuration; compliance is not guaranteed, especially for vague or conflicting instructions. Anything that must happen without exception should be a hook.
  Quote: "The more specific and concise your instructions, the more consistently Claude follows them."
- Claim 2: Guidance: write instructions concrete enough to verify ("Run npm test before committing" rather than "Test your changes"); target under 200 lines per file; imports do not reduce context cost; contradictory instructions may be resolved arbitrarily, so remove outdated or conflicting ones; Claude Code can read AGENTS.md in place of CLAUDE.md; a `/doctor prompt-audit` checks for references to files or commands that no longer exist.
- Bears on this repository: Rules for AGENTS.md that a command can check: line count under 200; every command named in it exists in `BUILD.bazel`/CI (a script runs each `bazel` target named); no two statements of the same rule.

### Claude Code, best practices
- Citation: Anthropic, "Best practices for Claude Code". https://code.claude.com/docs/en/best-practices
- Kind: primary (vendor guidance)
- Read: 2026-11-25, in part (sections on CLAUDE.md length, verification, hooks versus instructions)
- Topics: h
- Claim 1: Keep the file to what Claude cannot work out from the code: exact test, typecheck and lint commands (and how to run one test), places where the code is misleading, decisions not in the repository, anything corrected twice. Long files make lines get lost, and emphasis such as IMPORTANT will probably not fix a line that is ignored.
  Quote: "Claude reads the repo itself. Only write down what isn’t in there."
- Claim 2: Give the agent a check to run and ask for evidence (the command and its output); rules with zero exceptions belong in hooks, deny rules or CI, since "whether Claude follows that line on a given turn is still Claude’s call". It also warns that a reviewer prompted to find problems will always find something, so say what to skip.
- Bears on this repository: Supports "the first line of AGENTS.md is the command that runs the gate test"; rules like "control code reads sensors only" should be enforced by a Bazel `visibility`/dependency check, not left as prose.

### OpenAI Codex, custom instructions with AGENTS.md
- Citation: OpenAI, "Custom instructions with AGENTS.md" (Codex guide). https://developers.openai.com/codex/guides/agents-md
- Kind: primary (vendor documentation)
- Read: 2026-11-25, whole of the article body
- Topics: h
- Claim 1: Codex builds the instruction chain once per run: global file, then each directory from the project root down to the working directory, one file per directory (AGENTS.override.md, then AGENTS.md, then configured fallbacks), concatenated from root down.
- Claim 2: Combined size is capped; content past the cap is dropped, and the page tells users to raise the limit or split into nested directories when instructions are truncated.
  Quote: "stops adding files once the combined size reaches the limit defined by project_doc_max_bytes (32 KiB by default)"
- Bears on this repository: A checkable rule: AGENTS.md (plus any other file read with it) stays well under 32 KiB, so nothing is silently truncated in Codex.

### GitHub Copilot, repository custom instructions
- Citation: GitHub, "Adding repository custom instructions for GitHub Copilot" (GitHub Docs). https://docs.github.com/en/copilot/how-tos/configure-custom-instructions/add-repository-instructions
- Kind: primary (vendor documentation)
- Read: 2026-11-25, in part (repository-wide and path-specific sections, the onboarding prompt)
- Topics: h
- Claim 1: Repository-wide instructions are `.github/copilot-instructions.md`; path-specific instructions are `.github/instructions/NAME.instructions.md` with an `applyTo` glob; Copilot also reads AGENTS.md anywhere in the repository, nearest wins. Docs advise avoiding conflicting sets of instructions.
- Claim 2: The page's own prompt for generating an instructions file asks for "no longer than 2 pages", not task-specific, and for build, test, run and lint commands that have each been run and validated, with timings of commands that timed out. The vendor states this will "SIGNIFICANTLY improve" agent output; no measurement is given.
  Quote: "Instructions must be no longer than 2 pages."
- Bears on this repository: Supports documenting validated commands with versions and known failure notes (GCC only, Bazel 9); the claim of improvement is a vendor assertion and is contradicted in part by the Gloaguen paper below.

### Cursor, rules
- Citation: Cursor, "Rules" (Cursor docs). https://cursor.com/docs/context/rules
- Kind: primary (vendor documentation)
- Read: 2026-11-25, in part (best practices, what to avoid, AGENTS.md)
- Topics: h
- Claim 1: Rules should be focused, actionable and scoped; under 500 lines; split large rules; reference files rather than copying contents, which keeps rules short and prevents staleness. It also supports AGENTS.md (root and nested) as a plain alternative.
  Quote: "Reference files instead of copying their contents."
- Claim 2: What to avoid: copying whole style guides (use a linter), documenting every command, edge cases that rarely apply, duplicating what is in the code; add rules only when the agent repeats the same mistake.
- Bears on this repository: Supports leaving style to `clang-format`/`clang-tidy` and pointing AGENTS.md to a canonical file instead of copying; "add a rule after a repeated mistake" is a process rule, not checkable.

### Evaluating AGENTS.md
- Citation: Thibaud Gloaguen et al., "Evaluating AGENTS.md: Are Repository-Level Context Files Helpful for Coding Agents?", arXiv:2602.11988 (v3), Feb 2026. https://arxiv.org/abs/2602.11988 (full text read at https://arxiv.org/html/2602.11988v3)
- Kind: primary (the paper)
- Read: 2026-11-25, in part (abstract, setup, results, analysis, limitations, conclusion; appendices skimmed)
- Topics: h
- Claim 1: Measured: four agent/model pairs (Claude Code with Sonnet-4.5; Codex with GPT-5.2 and GPT-5.1 mini; Qwen Code with Qwen3-30b-coder) on SWE-bench Lite (300 tasks, 11 Python repositories, none with context files) and a new CTXbench (138 instances from 12 Python repositories that have developer-written context files), each with no file, an LLM-generated file, and the developer's file. Success rate with LLM-generated files fell by 0.5% (SWE-bench) and 2% (CTXbench) on average, not significant; developer files raised it by 2.4% on average (p=21%, not significant). Cost rose by about 20% (LLM files) and 23%, up to 19% for developer files, with 2 to 4 more steps.
  Quote: "providing context files does not generally improve task success rates, while increasing inference cost by over 20% on average"
- Claim 2: Trace analysis: instructions in the files are followed (more testing and exploration, hence cost), but repository overviews do not help the agent reach the relevant files sooner; 100% of Sonnet-4.5-generated files contained an overview. Context-file length and removal of instruction categories had no strong effect (appendix B). Conclusion: such files should only contain specific additional instructions beyond what is in the codebase. Limitations: Python only, the authors note popular languages may be in the models' training data; niche toolchains might behave differently.
- Bears on this repository: Do not put a directory tour in AGENTS.md; keep only commands and non-obvious rules; result is for Python, so for a C++/Bazel repo it is untested, and any AGENTS.md change should be evaluated (e.g. gate-test pass rate and cost on a fixed set of tasks), as the authors urge.

### Agent READMEs (arXiv 2511.12884)
- Citation: Worawalan Authors (submitter Yutaro Kashiwa), "Agent READMEs: An Empirical Study of Context Files for Agentic Coding", arXiv:2511.12884 (v2), Nov 2025. https://arxiv.org/abs/2511.12884 (full text at https://arxiv.org/html/2511.12884v2)
- Kind: primary (the paper)
- Read: 2026-11-25, in part (abstract, method, maintenance and content results)
- Topics: h
- Claim 1: Measured, not tested for effect: 2,303 context files from 1,925 GitHub repositories (CLAUDE.md, AGENTS.md, copilot-instructions.md). Median length 535 words (Copilot), 485 (Claude Code), 335.5 (Codex); Claude Code files are hard to read (Flesch median 42.39). Files change by frequent small additions: 67.0% of Claude Code files were modified in multiple commits; in 100 sampled commits, 92% were substantive instruction edits.
- Claim 2: Content of 16 instruction types: testing 75.9%, implementation details 70.8%, architecture 68.1%, build and run 63.0%, system overview 59.0%; security only 14.8% and performance 14.5%.
- Bears on this repository: Describes what people write, not what works; use only as the baseline for what an AGENTS.md usually holds. A rule requiring a short "non-functional" section (determinism, sensor-only control) addresses the gap the authors note.

### Lost in the Middle
- Citation: Liu et al., "Lost in the Middle: How Language Models Use Long Contexts", arXiv:2307.03172, TACL 2023. https://arxiv.org/abs/2307.03172 (full text at https://arxiv.org/html/2307.03172)
- Kind: primary (the paper)
- Read: 2026-11-25, in part (abstract, introduction, multi-document QA results)
- Topics: h
- Claim 1: Measured: multi-document question answering and synthetic key-value retrieval, with MPT-30B-Instruct, LongChat-13B (16K), GPT-3.5-Turbo (4K and 16K) and Claude-1.3, varying the position of the relevant passage. Accuracy is highest when the relevant item is first or last and drops in the middle (U-shape); with GPT-3.5-Turbo, middle placement scored below its closed-book (no documents) accuracy of 56.1%.
  Quote: "performance is often highest when relevant information occurs at the beginning or end of the input context"
- Claim 2: Using 50 retrieved documents instead of 20 gained only about 1.5% (GPT-3.5-Turbo) and 1% (Claude-1.3): more context is not used proportionally. These are 2023 models and a retrieval task, not instruction following by coding agents.
- Bears on this repository: Weak support for putting the most important commands and prohibitions at the top (and the end) of AGENTS.md and keeping the file short; not a measurement of agents.

### IFScale (arXiv 2507.11538)
- Citation: Jaroslawicz et al. (submitter Daniel Jaroslawicz), "How Many Instructions Can LLMs Follow at Once?", arXiv:2507.11538, July 2025. https://arxiv.org/abs/2507.11538 (full text at https://arxiv.org/html/2507.11538v1)
- Kind: primary (the paper)
- Read: 2026-11-25, in part (abstract, results, primacy and error-type sections)
- Topics: h
- Claim 1: Measured: 20 models from 7 providers on IFScale, a business-report writing task with 10 to 500 keyword-inclusion instructions, 5 runs each, scored by pattern matching. Best accuracy at 500 instructions is 68% (frontier models); degradation shape differs: threshold decay (o3, gemini-2.5-pro), linear (gpt-4.1, Claude Sonnet models), exponential (gpt-4o, llama-4-scout).
  Quote: "even the best frontier models only achieve 68% accuracy at the max density of 500 instructions"
- Claim 2: Bias toward earlier instructions peaks around 150 to 200 instructions; the dominant error type is omission (dropping the instruction) rather than altering it. Task is trivial instructions (include a word), so it shows a trend with instruction count, not a safe limit for coding rules.
- Bears on this repository: Supports few, ordered, high-priority rules, and moving rules a tool can check into tools (clang-tidy, dependency check) so they do not compete for instruction budget.

### Agentic refactoring in the wild
- Citation: Authors at Nara Institute of Science and Technology (names not recorded), "Agentic Refactoring: An Empirical Study of AI Coding Agents", arXiv:2511.04824 (v2), accepted to ACM TOSEM. https://arxiv.org/abs/2511.04824 (full text at https://arxiv.org/html/2511.04824v2)
- Kind: primary (the paper)
- Read: 2026-11-25, in part (abstract, introduction, discussion/implications; method skimmed)
- Topics: h
- Claim 1: Measured: 51,412 refactoring instances in 12,256 pull requests and 14,998 commits by agents (Codex, Claude Code, Cursor and others, from the AIDev dataset) in open-source Java projects, detected with RefactoringMiner and scored with DesigniteJava metrics. Agents explicitly target refactoring in 26.1% of commits; most common operations Rename Variable (7.6%) and Change Variable Type (6.8%); quality gains in metrics are small though statistically significant (class size, complexity).
- Claim 2: The study counts and classifies refactorings; it does not establish that behavior was preserved. It notes 36.6% of refactoring instances sit in commits whose messages do not mention refactoring (tangled changes), making behavior preservation harder to verify, and recommends telling agents via context files to avoid mixing refactoring with other changes.
- Bears on this repository: Rule candidate: a refactoring change is its own commit/PR, with no behavior change mixed in; the gate test and unit tests run before and after, and the paper gives no evidence that agent refactorings are safe without that.

### Differential fuzzing of LLM refactorings
- Citation: Authors not recorded, "A Differential Fuzzing-Based Evaluation of Functional Equivalence in LLM-Generated Code Refactorings", arXiv:2602.15761, Feb 2026. https://arxiv.org/abs/2602.15761
- Kind: primary (the paper)
- Read: 2026-11-25, in part (abstract only; I did not read the body)
- Topics: h
- Claim 1: Measured: six LLMs (CodeLlama, Codestral, StarChat2, Qwen-2.5, Olmo-3, GPT-4o), three datasets, two refactoring types; 19 to 35% of refactorings were functionally non-equivalent when checked by differential fuzzing, and about 21% of those non-equivalent refactorings were not caught by the datasets' existing test suites. These are single-function refactorings by non-agentic LLMs, not whole-repository agents.
  Quote: "reliance on existing tests might overestimate functional equivalence in LLM-generated code refactorings"
- Bears on this repository: Argues that "tests still pass" is not enough evidence for "without breaking behavior"; for this repo a golden-output check (the programs that print CSV tables compared against stored outputs before and after) is the closest cheap equivalent.

### CodeTaste (arXiv 2603.04177)
- Citation: "CodeTaste: Can LLMs Generate Human-Level Code Refactorings?", arXiv:2603.04177, March 2026 (authors not recorded). https://arxiv.org/abs/2603.04177 (full text at https://arxiv.org/html/2603.04177v1)
- Kind: primary (the paper)
- Read: 2026-11-25, in part (abstract, introduction, benchmark construction, results)
- Topics: h
- Claim 1: Measured: 100 refactoring tasks mined from 87 repositories in 6 languages (average 91.5 files and 2,605 lines changed); four agents (GPT-5.2, GPT-5.1 Codex Mini, Claude Sonnet 4.5, Qwen3 Coder 30B) scored by repository tests (average 1,638 tests) plus static-analysis rules checking that undesired patterns are gone and desired ones present. With a detailed specification, GPT-5.2 reached 69.6% alignment (test pass 76.0%), Sonnet 4.5 32.4%; when only a focus area was given, alignment was at most 7.7% (14.1% with a propose-then-implement plan for GPT-5.2).
  Quote: "agents perform well at implementing refactorings that are specified in detail, but often fail to discover the human refactoring choices"
- Claim 2: Safety was checked by tests and by static rules together; test passing alone does not show the intended transformation was done. Cost cap of 11 dollars per task; GPT-5.2 hit it in 14 instances.
- Bears on this repository: Supports telling the agent a specific refactoring plus a machine check of the target pattern (e.g. a `grep`/clang-tidy check that the old pattern is gone) rather than "improve the repo"; supports a propose-then-implement step where the owner approves the list first.

## Does not fit

- Diátaxis as a four-part documentation tree (Procida, "Diátaxis"): its tutorial and how-to quadrants serve outside learners and users; this repository has one owner and agent contributors, so a full four-way tree is overhead. Use reference and explanation only.
- Full MADR with option pros and cons, consulted and informed fields, and a review meeting with a dedicated reading slot (MADR 4.0.0 template; AWS ADR process): these assume a team that debates; here a short Nygard-style record plus a Confirmation line is enough.
- Nested per-directory AGENTS.md files (agents.md, Codex guide, Cursor rules): written for monorepos; this repository has a flat `src/` and one Bazel package, and nested files would only create duplicate statements (against DRY).
- Generated "repository overview" sections in AGENTS.md, as produced by `/init` or the Copilot onboarding prompt (arXiv:2602.11988; Copilot docs): measured as not helping the agent find files, while adding cost; the stand-in cell will also be replaced, so a tour goes stale.
- Freshness dates with owner reminders by email (SWE at Google, ch. 10): needs infrastructure and several owners; for one owner, deletion of dead documents on the change that obsoletes them (docguide) is the cheaper rule.

## Leads not followed

- Zimmermann, "Ten Common Mistakes in Architectural Decision Records" and "Seven Architectural Decision Making Fallacies": cited as new in the 2023 post's update note; not fetched. The earlier ADRCreation URL guess (/practices/2023/12/01/ADRCreationAntiPatterns.html) returned 404, so no link is given.
- Zimmermann's Medium posts on MADR and Y-statements (medium.com/olzzio/...): returned 403; Y-statement template not read first-hand.
- willhaben "8 Learnings from using Architecture Decision Records" (cited in Zimmermann's post) and Spotify's "When should I write an ADR": team retrospectives, not opened.
- Wilson et al., "Good Enough Practices in Scientific Computing", arXiv:1609.00037 (for scripted, reproducible figures): abstract fetched, PDF text extraction was too garbled to quote, so not cited.
- Authors not recorded, 2025, "On the Use of Agentic Coding Manifests", arXiv:2509.14744 (253 CLAUDE.md files, 242 repositories): abstract only; superseded by the larger study above.
- SWE-Refactor (arXiv:2602.03712) and RefAgent (arXiv:2511.03153): seen in a search result only, Java benchmarks.
- Diataxis pages on reference and explanation, Anthropic engineering post on agent context ("effective context engineering"), SWE-bench repository-feature studies (what repository properties raise agent resolve rate): no source found and read on that exact question; topic (h) "tests that run fast, small files" remains supported only by vendor advice (Claude best practices: give a check to run), not by a paper I read.
