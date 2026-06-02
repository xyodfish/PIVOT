#!/usr/bin/env python3
"""Build a launch YAML for robot_kinematic_viewer (stdout path or --output)."""

from __future__ import annotations

import argparse
import os
import sys
import tempfile
from pathlib import Path

try:
    import yaml
except ImportError as exc:
    sys.stderr.write(
        "rkv_prepare_config.py requires PyYAML (pip install pyyaml)\n"
    )
    raise SystemExit(1) from exc


def _load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"expected mapping in {path}")
    return data


def _repo_relative(path: Path, repo_root: Path) -> str:
    try:
        return str(path.resolve().relative_to(repo_root.resolve()))
    except ValueError:
        return str(path.resolve())


def prepare_config(
    base: Path,
    repo_root: Path,
    *,
    urdf: str | None = None,
    trajectories: list[str] | None = None,
    title_suffix: str | None = None,
    point_cloud_enable: bool = False,
    point_cloud_file: str | None = None,
    point_cloud_auto_load: bool = True,
    point_cloud_build_esdf: bool = False,
) -> dict:
    cfg = _load_yaml(base)

    if urdf:
        cfg.setdefault("robot", {})["urdf_path"] = str(Path(urdf).resolve())

    if trajectories:
        files: list[str] = []
        for t in trajectories:
            p = Path(t)
            if not p.is_absolute():
                p = (repo_root / p).resolve()
            if not p.is_file():
                raise FileNotFoundError(f"trajectory not found: {p}")
            files.append(_repo_relative(p, repo_root))
        cfg.setdefault("playback", {})
        cfg["playback"]["trajectory_files"] = files
        cfg["playback"]["selected_index"] = 0

    if title_suffix:
        window = cfg.setdefault("window", {})
        title = window.get("title", "机器人运动学调试界面")
        if title_suffix not in title:
            window["title"] = f"{title} — {title_suffix}"

    if point_cloud_enable or point_cloud_file:
        pc = cfg.setdefault("point_cloud", {})
        pc["enable"] = True
        pc["visible"] = True
        pc["auto_load_on_start"] = point_cloud_auto_load
        if point_cloud_file:
            p = Path(point_cloud_file)
            if not p.is_absolute():
                p = (repo_root / p).resolve()
            pc["file_path"] = _repo_relative(p, repo_root)
        if point_cloud_build_esdf:
            pc["build_esdf"] = True
            pc["color_mode"] = "flat"
            pc["esdf_visual_mode"] = "occupied"
            pc["esdf_color_mode"] = "height"
            pc["point_size_px"] = 6.0

    return cfg


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", required=True, type=Path, help="source YAML")
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=None,
        help="repository root for relative trajectory paths (default: parent of config/)",
    )
    parser.add_argument("--urdf", default=None, help="override robot.urdf_path")
    parser.add_argument(
        "--trajectory",
        action="append",
        dest="trajectories",
        default=None,
        help="preload trajectory CSV (repeatable; replaces playback list)",
    )
    parser.add_argument("--title-suffix", default=None)
    parser.add_argument("--point-cloud-enable", action="store_true")
    parser.add_argument("--point-cloud-file", default=None)
    parser.add_argument("--no-point-cloud-autoload", action="store_true")
    parser.add_argument(
        "--point-cloud-build-esdf",
        action="store_true",
        help="build ESDF from PCD on load (occupancy + EDT)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="write YAML here; default: temp file under /tmp",
    )
    args = parser.parse_args()

    repo_root = args.repo_root
    if repo_root is None:
        repo_root = args.base.resolve().parent.parent

    cfg = prepare_config(
        args.base,
        repo_root,
        urdf=args.urdf,
        trajectories=args.trajectories,
        title_suffix=args.title_suffix,
        point_cloud_enable=args.point_cloud_enable,
        point_cloud_file=args.point_cloud_file,
        point_cloud_auto_load=not args.no_point_cloud_autoload,
        point_cloud_build_esdf=args.point_cloud_build_esdf,
    )

    out = args.output
    if out is None:
        fd, name = tempfile.mkstemp(prefix="rkv_config_", suffix=".yaml")
        os.close(fd)
        out = Path(name)

    out = out.resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)

    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
