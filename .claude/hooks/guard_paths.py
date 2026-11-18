"""PreToolUse hook for Edit / Write / MultiEdit / NotebookEdit (AGENT_PLAN.md 2.1). Exit 2 blocks."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import guard_lib as g  # noqa: E402


def main():
    data = g.read_hook_input(sys.stdin)
    ti = data.get("tool_input") or {}
    path = ti.get("file_path") or ti.get("notebook_path") or ti.get("path")
    if not path:
        return 0
    root = os.environ.get("CLAUDE_PROJECT_DIR") or g.DEFAULT_ROOT
    rel = g.rel_path(path, root, data.get("cwd") or root)
    reason = g.path_block_reason(rel, root, data.get("agent_type"))
    if reason:
        print("BLOCKED by .claude/hooks/guard_paths.py: " + reason, file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
