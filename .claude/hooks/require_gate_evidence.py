"""SubagentStop hook for integration-test-writer (AGENT_PLAN.md 2.3).

Blocks the writer's stop unless its last message ends with the gate's last line ("GATE PASS <path>" or
"GATE FAIL ...") and, for PASS, the evidence file exists under ../ffb-test-build/it-gate/ and every file it
records still has the blob the gate tested. With stop_hook_active set it lets the stop through (no loop);
the lead then treats the result as unverified."""
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import guard_lib as g  # noqa: E402


def last_assistant_text(data):
    msg = data.get("last_assistant_message")
    if isinstance(msg, str) and msg.strip():
        return msg
    path = data.get("agent_transcript_path") or data.get("transcript_path")
    text = ""
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                try:
                    rec = json.loads(line)
                except ValueError:
                    continue
                m = rec.get("message") or {}
                if rec.get("type") != "assistant" and m.get("role") != "assistant":
                    continue
                content = m.get("content")
                if isinstance(content, str):
                    parts = [content]
                else:
                    parts = [c.get("text", "") for c in content or [] if isinstance(c, dict) and c.get("type") == "text"]
                if any(p.strip() for p in parts):
                    text = "\n".join(parts)
    except (OSError, TypeError):
        pass
    return text


def check(data, root):
    """Reason to block, or None."""
    if data.get("agent_type") and data.get("agent_type") != g.WRITER:
        return None
    if data.get("stop_hook_active"):
        return None
    text = last_assistant_text(data).rstrip()
    last = text.splitlines()[-1].strip() if text else ""
    if last.startswith("GATE FAIL"):
        return None
    m = re.match(r"^GATE PASS (\S+)$", last)
    if not m:
        return ("end your answer with the last line of it_gate.py ('GATE PASS <evidence path>' or 'GATE FAIL ...'); "
                "run: python3 -B Firmware/Tests/Integration_Tests/tools/it_gate.py --wo <ID> --variants default,m32")
    evp = os.path.realpath(m.group(1))
    if not evp.startswith(g.gate_dir(root) + os.sep):
        return "the evidence path %s is not under %s" % (evp, g.gate_dir(root))
    try:
        with open(evp, encoding="utf-8") as f:
            ev = json.load(f)
    except (OSError, ValueError):
        return "cannot read the gate evidence %s" % evp
    if ev.get("result") != "PASS":
        return "the evidence %s does not record PASS" % evp
    files = ev.get("files") or {}
    for rel, blob in files.items():
        if g.blob_of_file(root, rel) != blob:
            return "%s changed after the gate ran; run the gate again" % rel
    if ev.get("fingerprint") and ev["fingerprint"] != g.fingerprint(files):
        return "the evidence fingerprint does not match its file list"
    return None


def main():
    data = g.read_hook_input(sys.stdin)
    root = os.environ.get("CLAUDE_PROJECT_DIR") or g.DEFAULT_ROOT
    reason = check(data, root)
    if reason:
        print(json.dumps({"decision": "block", "reason": reason}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
