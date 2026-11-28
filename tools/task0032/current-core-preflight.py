#!/usr/bin/env python3
"""Requalify archived small-core transformations on the current Drive candidate.

This wrapper does not change the historical generator or its numeric oracles.
Only a detached, clean, exact source revision may be used. The output patch
is a disposable counterfactual, never an update to the P0 branch.
"""
from pathlib import Path
import runpy
import subprocess
import sys

P0 = "95415cff552517a40a844cc1935f588baf9a5069"
HERE = Path(__file__).resolve().parent

def main():
    # Keep the original argument parser and every exact-source anchor/check.
    # No relaxation of the old numeric oracle or fixture assertions.
    original = runpy.run_path(str(HERE / "combined-core-preflight.py"))
    original["main"].__globals__["P0"] = P0
    original["main"]()

if __name__ == "__main__":
    main()
