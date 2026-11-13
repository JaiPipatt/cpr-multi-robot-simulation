"""ANSI color-coded console logging."""
import os
import sys

os.system("")  # enable ANSI escape handling on Windows terminals

_ENABLED = sys.stdout.isatty() and "NO_COLOR" not in os.environ

RESET = "\033[0m"
COLORS = {
    "info": "\033[90m",     # gray   - step ticks
    "state": "\033[36m",    # cyan   - robot state changes
    "consensus": "\033[35m",  # magenta - paxos decisions
    "msg": "\033[34m",      # blue   - broadcasts
    "success": "\033[32m",  # green  - goal picked
    "warn": "\033[33m",     # yellow - invalid input
    "error": "\033[1;31m",  # bold red - illegal behavior
}


def log(kind, text):
    if _ENABLED:
        print(f"{COLORS[kind]}{text}{RESET}")
    else:
        print(text)
