"""check_report: one layout's checker report as a file, made once per layout and checker build.

The layouts with "one_posture_per_module" take 125 to 180 s each to check, and
every shard of plan_replay_staged_test and every scorecard of scorecard_test
made their reports again (run 039). This rule runs

    check_layout <name> --report-out check_reports/<name>.json

in an action whose inputs are the layout's file, the model files and the
check_layout program alone: not the other layouts, not the program's runfiles.
So Bazel's disk and remote caches keep the report across commits that change
neither this layout nor anything check_layout is built from, and a test takes
the file as data (src/layout_report_file.h reads it, and refuses a file made
from another version of the layout).

The program is built in the target configuration (cfg = "target"), the one the
tests are built in, so MuJoCo and the checker are not compiled a second time
for the exec configuration, and the report is made by the same build of the
checker that the tests would run.

The model files are linked into a tree laid out as the program's runfiles
would be (<tree>/_main/models/..., <tree>/<repo>/... for an outside file such
as Menagerie's UR10e), because MuJoCo finds a model's meshes relative to the
scene: the generated meshes and the committed scenes must sit in one directory.
"""

def _check_report_impl(ctx):
    out = ctx.actions.declare_file("check_reports/%s.json" % ctx.attr.layout_name)
    layout = ctx.file.layout
    models = ctx.files.models
    tree = out.path + ".tree"
    lines = [
        "set -eu",
        "root=$PWD",
        "rm -rf %s" % tree,
    ]
    for f in models:
        dest = "%s/_main/%s" % (tree, f.short_path)
        lines.append("mkdir -p \"$(dirname %s)\" && ln -s \"$root/%s\" %s" % (dest, f.path, dest))
    lines.append("mkdir -p %s/_main/layouts" % tree)
    lines.append("ln -s \"$root/%s\" %s/_main/layouts/%s.json" % (layout.path, tree, ctx.attr.layout_name))
    lines.append(
        "LAYOUT_DIR=%s/_main/layouts MODELS_DIR=%s/_main/models UR10E_XML=%s/_main/%s %s %s --report-out %s" % (
            tree,
            tree,
            tree,
            ctx.file.ur10e.short_path,
            ctx.executable.tool.path,
            ctx.attr.layout_name,
            out.path,
        ),
    )
    lines.append("rm -rf %s" % tree)
    ctx.actions.run_shell(
        inputs = [layout, ctx.executable.tool] + models,
        outputs = [out],
        command = "\n".join(lines),
        mnemonic = "CheckReport",
        progress_message = "Checking layout %s once for its report" % ctx.attr.layout_name,
    )
    return [DefaultInfo(files = depset([out]), runfiles = ctx.runfiles(files = [out]))]

check_report = rule(
    implementation = _check_report_impl,
    doc = "The checker's report of one layout, check_reports/<layout_name>.json.",
    attrs = {
        "layout_name": attr.string(mandatory = True),
        "layout": attr.label(mandatory = True, allow_single_file = [".json"]),
        "models": attr.label_list(allow_files = True),
        "ur10e": attr.label(mandatory = True, allow_single_file = True),
        "tool": attr.label(mandatory = True, executable = True, cfg = "target"),
    },
)
