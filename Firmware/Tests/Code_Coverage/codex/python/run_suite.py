"""Run the Codex stdlib suite, rejecting an empty mandatory workload."""
import argparse
import sys
import unittest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', required=True)
    args = parser.parse_args()
    try:
        suite = unittest.defaultTestLoader.discover(args.start, pattern='test_*.py', top_level_dir=args.start)
    except (ImportError, OSError) as error:
        print('Codex test discovery failed: ' + str(error), file=sys.stderr)
        return 1
    if suite.countTestCases() == 0:
        print('Codex test discovery found no tests', file=sys.stderr)
        return 1
    return 0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1


if __name__ == '__main__':
    sys.exit(main())
