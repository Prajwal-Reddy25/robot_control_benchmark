"""Command-line interface for headless, reproducible benchmarks."""

from __future__ import annotations

import argparse
from pathlib import Path

from .benchmark import run_suite


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--config", required=True, help="benchmark YAML")
    result.add_argument(
        "--output", required=True, help="fresh output directory; matching artifacts are overwritten"
    )
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    results = run_suite(args.config, args.output)
    report = Path(args.output) / "REPORT.md"
    print(f"Completed {len(results)} deterministic trials; report: {report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

