# Citation check C1: sources S1 to S20

All 20 sources were opened on 2026-11-25 with curl (HTML converted to text; the two PDFs, S2 and S9, extracted with a small zlib script). Quotations were matched word for word after ignoring whitespace, line breaks and typographic quotes. Claims counted below are the sentence-level claims in each source entry; quotations are the double-quoted strings in the entry (and in the body where they cite the source).

Totals: 20 opened, 15 OK, 5 corrected (S6, S10, S11, S14, S17), 0 unreachable. About 45 claims in entries, 17 quotations (all word for word), about 60 citing sentences in the body checked; 7 corrections.

## S1 OK
Fetched https://blog.acolyer.org/2016/09/05/... Secondary is right. 1 claim, 1 quotation (exact). Body line 260: ok. Note: the page's heading says "David L Parnas, 1971", the entry says CACM 1972; the paper itself is CACM December 1972, so no change.

## S2 OK
Fetched https://web.archive.org/web/2007/http://www.objectmentor.com/resources/articles/granularity.pdf (PDF, text extracted). 2 claims, 1 quotation (exact: "Thus the dependency structure must always be monitored for cycles."). The page says the cycle makes MyApplication, MyTasks, MyDialogs "must always be released at the same time". Body line 261: ok. The extracted text does not show the author line; the Object Mentor/Martin attribution rests on the URL.

## S3 OK
Fetched https://abseil.io/resources/swe-book/html/ch18.html. 2 claims, 1 quotation (exact). Visibility "as narrow as possible" is a paraphrase of "it is usually best to minimize visibility as much as possible" (not quoted). Body line 262: ok.

## S4 OK
Fetched https://bazel.build/concepts/visibility. 2 claims, 0 quotations. Default is "//visibility:private"; "fail to build during the analysis phase". Body lines 285, 296: ok.

## S5 OK
Fetched https://bazel.build/build/style-guide. 3 claims, 1 quotation (exact; the page shows a space before the final period only because of code markup). Direct deps only and "a BUILD file in each directory" confirmed. Body lines 285-287: ok.

## S6 CORRECTED
Fetched https://bazel.build/reference/be/c-cpp. 2 claims, 1 quotation (exact). Entry and body ok except line 611: "`layering_check` and `parse_headers` [S6]". The page has 0 occurrences of parse_headers. Correction in JSON (cite only layering_check). Body lines 288, 299: ok.

## S7 OK
Fetched raw README. Author "Martin Medler" confirmed in the licence line. 3 claims: aspect, unused deps, preprocessing step only ("it only executes the preprocessing step"), "not limited to LLVM based toolchains". Body lines 289, 301, 511, 526, 613: ok.

## S8 OK
Fetched raw README. 2 claims, 1 quotation (exact: "without generating a compile commands database"). The README pins a git commit and mentions no releases, so "no pinned release" is an inference from absence; accepted. Body lines 289, 613: ok.

## S9 OK
Fetched https://aisberg.unibg.it/bitstream/10446/87662/1/78-379-1-PB.pdf (PDF, text extracted; the "fi" ligature comes out as a control code, so "Configuration" was matched with that allowed). 3 quotations (exact): "should be kept fully separated"; "Configuration influences the behavior of entities of the other concerns by changing its settings."; "a recipe for failure in the long term" (the paper says this of imposing a specific architecture from the start, as line 308 says). Body lines 308, 325, 641: ok. Soft point, no correction: line 325 cites [S9] after a sentence about repo files; the paper supports only the separation of configuration.

## S10 CORRECTED
Fetched raw rep-0103.rst. Authors "Tully Foote, Mike Purvis". 3 claims, 1 quotation (exact). Failures in the body: line 311 says REP 103 asks that another convention "be named and justified"; REP says "well justified and well documented". Line 327 says REP 103's "named-suffix practice" covers degrees in a JSON field; the suffixes ("_optical", "_ned") name coordinate frames only. Two corrections in JSON.

## S11 CORRECTED
Fetched https://mujoco.readthedocs.io/en/stable/computation/index.html. 2 claims, 1 quotation (exact, section Reproducibility). Line 343 says the documentation "warns against" exact golden files; it never mentions golden files, it says exact reproducibility holds only within one version and architecture. One correction in JSON. Body lines 312, 338, 353, 636: ok.

## S12 OK
Fetched https://eel.is/c++draft/rand.dist.general and /rand.predef. 2 claims, 1 quotation (exact, clause 3). mt19937 has "Required behavior" (10000th value 4123659995). Body lines 313, 320: ok.

## S13 OK
Fetched the Rolling page. 2 claims, 1 quotation (exact). Mock selected in the URDF (mock components, "parsed from the URDF"). Body lines 308, 323: ok.

## S14 CORRECTED
Fetched https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html. 2 claims, 1 quotation (exact). The claim "Fused multiply-add changes rounding" (entry line 75 and body line 314) is not on the page: the entry says "-ffp-contract=fast enables floating-point expression contraction such as forming of fused multiply-add operations if the target has native support for them". Two corrections in JSON. Line 330, 353: ok.

## S15 OK
Fetched https://michaelfeathers.silvrback.com/characterization-testing (dated August 08, 2016). 2 claims, 1 quotation (exact). "This is particularly useful when we want to refactor or rewrite" and "if you haven't determined that the behavior you've uncovered is a bug, it's often a good idea to leave the test in place". Body lines 335, 349: ok.

## S16 OK
Fetched https://refactoring.com/. 2 claims, 1 quotation (exact; the page continues "Its heart is a series of small behavior preserving transformations"). The page says nothing on numeric output. Body line 336: ok.

## S17 CORRECTED
Fetched raw CustomComparators.md. 2 claims, 1 quotation (exact). The example is images differing in colour values or bit depth on different test environments (accepted for "varies by machine"). The second page named in the entry has no URL in the entry (not fetched; only the first was checked). Line 356 attributes "leave out columns of time" to S17, which does not say it; one correction in JSON. Lines 337, 343: ok.

## S18 OK
Fetched https://bazel.build/reference/test-encyclopedia. 3 claims, 0 quotations. size implies timeout; default "medium"; --test_verbose_timeout_warnings shows "tests whose specified size is too big". Body line 341: ok.

## S19 OK
Fetched https://www.cognitect.com/blog/2011/11/15/documenting-architecture-decisions (dated November 15, 2011). 3 claims, 1 quotation (exact). The experience report says "since early August", so "a few months" is fair. Body lines 361, 368: ok.

## S20 OK
Fetched https://martinfowler.com/bliki/ArchitectureDecisionRecord.html (24 March 2026). 3 claims, 1 quotation (exact). Body lines 363, 367, 378: ok.
