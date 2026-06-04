#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import platform
import xml.etree.ElementTree as ET
from pathlib import Path

from cocotb_tools.runner import get_runner


def read_filelist(path: Path) -> tuple[list[Path], list[Path]]:
    base = path.parent
    sources: list[Path] = []
    include_dirs: list[Path] = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        if line.startswith("+incdir+"):
            for include_dir in line[len("+incdir+") :].split("+"):
                if include_dir:
                    include_dirs.append((base / include_dir).resolve())
            continue
        if line.startswith("-I"):
            include_dirs.append((base / line[2:].strip()).resolve())
            continue
        sources.append((base / line).resolve())
    return sources, include_dirs


def configure_tool_environment() -> None:
    os.environ.pop("LC_ALL", None)
    os.environ["LANG"] = "en_US.UTF-8"
    os.environ["LC_CTYPE"] = "en_US.UTF-8"
    if platform.system() == "Darwin":
        path_parts = []
        for candidate in [
            Path("/opt/homebrew/bin"),
            Path.home() / "utils" / "oss-cad-suite" / "bin",
        ]:
            if candidate.is_dir():
                path_parts.append(str(candidate))
        path_parts.extend(["/usr/bin", "/bin", "/usr/sbin", "/sbin"])
        path_parts.append(os.environ.get("PATH", ""))
        os.environ["PATH"] = ":".join(part for part in path_parts if part)
        os.environ["AR"] = "/usr/bin/ar"
        os.environ["RANLIB"] = "/usr/bin/ranlib"
    elif (Path.home() / "utils" / "oss-cad-suite" / "bin").is_dir():
        os.environ["PATH"] = (
            f"{Path.home() / 'utils' / 'oss-cad-suite' / 'bin'}:"
            f"{os.environ.get('PATH', '')}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rtl", required=True, help="Path to stream_arb_fifo.sv")
    parser.add_argument("--tests", required=True, help="Path to CocoTB test module")
    parser.add_argument("--filelist", default=None, help="Optional Verilog source filelist")
    parser.add_argument(
        "--include-dir",
        action="append",
        default=[],
        help="Verilog include directory. May be passed multiple times.",
    )
    parser.add_argument("--build-dir", default="build/cocotb")
    parser.add_argument("--coverage-file", default="reports/coverage.json")
    parser.add_argument("--results-xml", default="reports/results.xml")
    parser.add_argument("--top", default="interrupt_controller_tb")
    args = parser.parse_args()

    rtl = Path(args.rtl).resolve()
    tests = Path(args.tests).resolve()
    filelist = Path(args.filelist).resolve() if args.filelist else None
    include_dirs = [Path(item).resolve() for item in args.include_dir]
    build_dir = Path(args.build_dir).resolve()
    coverage_file = Path(args.coverage_file).resolve()
    results_xml = Path(args.results_xml).resolve()

    coverage_file.parent.mkdir(parents=True, exist_ok=True)
    results_xml.parent.mkdir(parents=True, exist_ok=True)
    if coverage_file.exists():stream_arb_fifo
        coverage_file.unlink()

    configure_tool_environment()
    sources: list[Path] = []
    if filelist:
        sources, filelist_include_dirs = read_filelist(filelist)
        include_dirs.extend(filelist_include_dirs)
    if rtl not in sources:
        sources.append(rtl)
    runner = get_runner("verilator")
    runner.build(
        sources=sources,
        includes=include_dirs,
        hdl_toplevel=args.top,
        build_args=["--timing", "-Wno-fatal", "-Wno-WIDTHEXPAND"],
        build_dir=build_dir,
        always=True,
        clean=True,
    )
    runner.test(
        hdl_toplevel=args.top,
        test_module=tests.stem,
        test_dir=tests.parent,
        build_dir=build_dir,
        results_xml=str(results_xml),
        extra_env={
            **os.environ,
            "STREAM_ARB_COVERAGE_FILE": str(coverage_file),
        },
    )
    tree = ET.parse(results_xml)
    root = tree.getroot()
    suites = root.findall(".//testsuite")
    if root.tag == "testsuite":
        suites.append(root)
    failures = sum(int(suite.attrib.get("failures", "0")) for suite in suites)
    errors = sum(int(suite.attrib.get("errors", "0")) for suite in suites)
    tests = sum(
        int(suite.attrib.get("tests", str(len(suite.findall("testcase")))))
        for suite in suites
    )
    failures += len(root.findall(".//failure"))
    errors += len(root.findall(".//error"))
    if tests == 0 or failures or errors:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
