"""PreToolUse hook for Bash (AGENT_PLAN.md 2.2). Exit 2 blocks."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import guard_lib as g  # noqa: E402


def main():
    data = g.read_hook_input(sys.stdin)
    cmd = (data.get("tool_input") or {}).get("command") or ""
    if not cmd.strip():
        return 0
    root = os.environ.get("CLAUDE_PROJECT_DIR") or g.DEFAULT_ROOT
    reason = g.bash_block_reason(cmd, root, data.get("cwd") or root, data.get("agent_type"))
    if reason:
        print("BLOCKED by .claude/hooks/guard_bash.py: " + reason, file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
