#!/usr/bin/env python3
"""Generate README.md from data/*.yaml files."""

from __future__ import annotations

import argparse
import sys
from datetime import date
from pathlib import Path

import yaml

# ── Section registry ────────────────────────────────────────────────────────
# (yaml_key, display_name, heading_level, render_mode, subsection_heading_level)
# render_mode: "bullet", "other_awesome"
# subsection_heading_level: 6 means ######, 4 means ####, None means no subsections

SECTIONS = [
    ("simulators", "Simulators", 2, "bullet", 6),
    # -- "Libraries" divider inserted here --
    ("dynamics-simulation", "Dynamics Simulation", 3, "bullet", None),
    ("inverse-kinematics", "Inverse Kinematics", 3, "bullet", None),
    ("machine-learning", "Machine Learning", 3, "bullet", None),
    ("motion-planning", "Motion Planning and Control", 3, "bullet", 6),
    ("optimization", "Optimization", 3, "bullet", None),
    ("robot-modeling", "Robot Modeling", 3, "bullet", 6),
    ("robot-platform", "Robot Platform", 3, "bullet", None),
    (
        "reinforcement-learning",
        "Reinforcement Learning for Robotics",
        3,
        "bullet",
        None,
    ),
    ("slam", "SLAM", 3, "bullet", 4),
    ("vision", "Vision", 3, "bullet", None),
    ("fluid", "Fluid", 3, "bullet", None),
    ("grasping", "Grasping", 3, "bullet", None),
    ("humanoid-robotics", "Humanoid Robotics", 3, "bullet", None),
    ("multiphysics", "Multiphysics", 3, "bullet", None),
    ("math", "Math", 3, "bullet", None),
    ("etc", "ETC", 3, "bullet", None),
    ("other-awesome-lists", "Other Awesome Lists", 2, "other_awesome", None),
]

SECTION_DESCRIPTIONS: dict[str, str] = {
    "simulators": "Simulation environments for testing and developing robotic systems.",
    "dynamics-simulation": "Physics engines and rigid/soft body dynamics libraries. See also [Comparisons](COMPARISONS.md).",
    "inverse-kinematics": "Libraries for computing joint configurations from end-effector poses.",
    "machine-learning": "Machine learning frameworks and tools applied to robotics.",
    "motion-planning": "Libraries for robot motion planning, trajectory optimization, and control.",
    "optimization": "Numerical optimization solvers and frameworks used in robotics.",
    "robot-modeling": "Tools and formats for describing robot models.",
    "robot-platform": "Middleware and frameworks for building robot software systems.",
    "reinforcement-learning": "Reinforcement learning libraries commonly used in robotic control.",
    "slam": "Simultaneous Localization and Mapping libraries.",
    "vision": "Computer vision libraries for robotic perception.",
    "fluid": "Fluid dynamics simulation libraries.",
    "grasping": "Libraries and tools for robotic grasping and manipulation.",
    "humanoid-robotics": "Environments and models for humanoid robot research.",
    "multiphysics": "Frameworks for coupled multi-physics simulations.",
    "math": "Mathematics libraries for spatial algebra, Lie groups, and linear algebra.",
    "etc": "Other robotics-related tools and utilities.",
    "other-awesome-lists": "Related curated lists of robotics and AI resources.",
}

# Subsection ordering per section
SUBSECTION_ORDER = {
    "simulators": ["Free or Open Source", "Commercial", "Cloud"],
    "motion-planning": [None, "Motion Optimizer", "Nearest Neighbor", "3D Mapping"],
    "robot-modeling": [
        "Robot Model Description Format",
        "Utility to Build Robot Models",
    ],
    "slam": [None, "SLAM Dataset"],
}

# Parent→children for indented sub-entries
CHILDREN = {
    "machine-learning": {"Gymnasium": ["gym-dart", "gym-gazebo"]},
    "robot-platform": {"Linorobot": ["onine"]},
    "motion-planning": {"Cover-Tree": ["Faster cover trees"]},
}

# Entries rendered as a plain header line (not a library entry)
HEADER_ENTRIES = {"Utility Software"}

# Section using 2-space indented bullets (all entries)
INDENT_ALL = {"inverse-kinematics"}


def _load_yaml(path: Path) -> list[dict]:
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
        return data if isinstance(data, list) else []


def _sort_entries(
    entries: list[dict],
    sort_key: str,
    children_map: dict[str, list[str]],
) -> list[dict]:
    """Sort entries while preserving parent→child adjacency.

    Parents are sorted; each parent's children stay immediately after it
    in their original order. HEADER_ENTRIES keep their position relative
    to their children. Entries without _meta sort to the end for
    stars/last_commit modes.
    """
    if sort_key == "none":
        return entries

    # Build child set for quick lookup
    child_names: set[str] = set()
    for kids in children_map.values():
        child_names.update(kids)

    # Separate parents from children
    parents = [e for e in entries if e["name"] not in child_names]

    # Sort parents
    if sort_key == "name":
        parents.sort(key=lambda e: e["name"].lower())
    elif sort_key == "stars":
        parents.sort(
            key=lambda e: (e.get("_meta") or {}).get("stars") or 0,
            reverse=True,
        )
    elif sort_key == "last_commit":
        parents.sort(
            key=lambda e: (e.get("_meta") or {}).get("last_commit") or "",
            reverse=True,
        )

    # Rebuild with children after their parents
    result: list[dict] = []
    for parent in parents:
        result.append(parent)
        kid_names = children_map.get(parent["name"], [])
        if kid_names:
            for entry in entries:
                if entry["name"] in kid_names:
                    result.append(entry)

    return result


def _format_stars(count: int) -> str:
    if count >= 1000:
        return f"{count / 1000:.1f}k".replace(".0k", "k")
    return str(count)


def _activity_emoji(entry: dict) -> str:
    """Return activity indicator emoji based on last commit and archived status."""
    if entry.get("archived") or (entry.get("_meta") or {}).get("archived"):
        return "💀"
    meta = entry.get("_meta") or {}
    last_commit = meta.get("last_commit")
    if not last_commit:
        return ""
    try:
        commit_date = date.fromisoformat(str(last_commit))
    except (ValueError, TypeError):
        return ""
    days_ago = (date.today() - commit_date).days
    if days_ago <= 365:
        return "🟢"
    elif days_ago <= 730:
        return "🟡"
    else:
        return "🔴"


def _repo_url(entry: dict) -> str | None:
    """Return the repository URL if available."""
    if entry.get("github"):
        return f"https://github.com/{entry['github']}"
    if entry.get("gitlab"):
        return f"https://gitlab.com/{entry['gitlab']}"
    if entry.get("bitbucket"):
        return f"https://bitbucket.org/{entry['bitbucket']}"
    if entry.get("code_url"):
        return entry["code_url"]
    return None


# ── Bullet rendering ───────────────────────────────────────────────────────


def _render_bullet(entry: dict, indent: str = "") -> str:
    name = entry["name"]
    url = entry.get("url")
    desc = entry.get("description", "")
    meta = entry.get("_meta") or {}

    emoji = _activity_emoji(entry)
    parts = [f"{indent}*"]
    if emoji:
        parts.append(emoji)

    if url:
        parts.append(f"[{name}]({url})")
    else:
        parts.append(name)

    if desc:
        parts.append(f"- {desc}")

    repo = _repo_url(entry)
    if repo:
        stars = meta.get("stars")
        if stars is not None:
            parts.append(f"[⭐ {_format_stars(stars)}]({repo})")
        else:
            if entry.get("github"):
                label = "github"
            elif entry.get("gitlab"):
                label = "gitlab"
            elif entry.get("bitbucket"):
                label = "bitbucket"
            else:
                label = "code"
            parts.append(f"[[{label}]({repo})]")

    return " ".join(parts)


# ── Other awesome lists rendering ───────────────────────────────────────────


def _render_other_awesome(entries: list[dict]) -> str:
    lines = []
    for entry in entries:
        name = entry["name"]
        url = entry.get("url", "")
        desc = entry.get("description", "")
        if url:
            line = f"* [{name}]({url})"
        else:
            line = f"* {name}"
        if desc:
            line += f" - {desc}"
        lines.append(line)
    return "\n".join(lines)


# ── Section rendering ───────────────────────────────────────────────────────


def _render_section(
    key: str,
    entries: list[dict],
    mode: str,
    sub_heading_level: int | None,
    sort_key: str = "name",
) -> str:
    lines: list[str] = []

    if mode == "other_awesome":
        return _render_other_awesome(entries)

    indent_all = key in INDENT_ALL
    base_indent = "  " if indent_all else ""
    children_map = CHILDREN.get(key, {})

    child_names = set()
    for kids in children_map.values():
        child_names.update(kids)

    subsection_order = SUBSECTION_ORDER.get(key)
    if subsection_order:
        grouped: dict[str | None, list[dict]] = {s: [] for s in subsection_order}
        for entry in entries:
            sub = entry.get("_subsection")
            if sub in grouped:
                grouped[sub].append(entry)
            elif None in grouped:
                grouped[None].append(entry)

        first_sub = True
        for sub in subsection_order:
            sub_entries = grouped.get(sub, [])
            if not sub_entries and sub is not None:
                continue

            sub_entries = _sort_entries(sub_entries, sort_key, children_map)

            if sub is not None:
                if not first_sub or (first_sub and lines):
                    lines.append("")
                hashes = "#" * (sub_heading_level or 6)
                lines.append(f"{hashes} {sub}")

            lines.append("")
            for entry in sub_entries:
                if entry["name"] in child_names:
                    continue
                if entry["name"] in HEADER_ENTRIES:
                    lines.append(f"* {entry['name']}")
                    kids = children_map.get(entry["name"], [])
                    for sub_entry in sub_entries:
                        if sub_entry["name"] in kids:
                            lines.append(_render_bullet(sub_entry, indent="  "))
                    continue

                lines.append(_render_bullet(entry, indent=base_indent))
                kids = children_map.get(entry["name"], [])
                for kid_name in kids:
                    for sub_entry in sub_entries:
                        if sub_entry["name"] == kid_name:
                            lines.append(_render_bullet(sub_entry, indent="  "))

            first_sub = False
    else:
        sorted_entries = _sort_entries(entries, sort_key, children_map)
        for entry in sorted_entries:
            if entry["name"] in child_names:
                continue
            lines.append(_render_bullet(entry, indent=base_indent))
            kids = children_map.get(entry["name"], [])
            for kid_name in kids:
                for sub_entry in sorted_entries:
                    if sub_entry["name"] == kid_name:
                        lines.append(_render_bullet(sub_entry, indent="  "))

    return "\n".join(lines)


# ── Table of Contents ───────────────────────────────────────────────────────

TOC = """\
## Contents
* [Simulators](#simulators)
* [Libraries](#libraries)
  * [Dynamics Simulation](#dynamics-simulation)
  * [Inverse Kinematics](#inverse-kinematics)
  * [Machine Learning](#machine-learning)
  * [Motion Planning and Control](#motion-planning-and-control)
  * [Optimization](#optimization)
  * [Robot Modeling](#robot-modeling)
  * [Robot Platform](#robot-platform)
  * [Reinforcement Learning for Robotics](#reinforcement-learning-for-robotics)
  * [SLAM](#slam)
  * [Vision](#vision)
  * [Fluid](#fluid)
  * [Grasping](#grasping)
  * [Humanoid Robotics](#humanoid-robotics)
  * [Multiphysics](#multiphysics)
  * [Math](#math)
  * [ETC](#etc)
* [Other Awesome Lists](#other-awesome-lists)"""


# ── Main ────────────────────────────────────────────────────────────────────


def generate(data_dir: Path, sort_key: str = "name") -> str:
    out: list[str] = []

    out.append("# Awesome Robotics Libraries")
    out.append("")
    out.append("[![Awesome](https://awesome.re/badge.svg)](https://awesome.re)")
    out.append("")
    out.append("A curated list of robotics simulators and libraries.")
    out.append("")
    out.append(TOC)
    out.append("")
    out.append(
        "> **Legend**: "
        "🟢 Active (<1yr) · "
        "🟡 Slow (1-2yr) · "
        "🔴 Stale (>2yr) · "
        "💀 Archived"
    )
    out.append("")

    libraries_emitted = False

    for key, display, level, mode, sub_hl in SECTIONS:
        yaml_path = data_dir / f"{key}.yaml"
        if not yaml_path.exists():
            continue
        entries = _load_yaml(yaml_path)
        if not entries:
            continue

        # Insert "Libraries" divider before first level-3 section
        if level == 3 and not libraries_emitted:
            out.append("## [Libraries](#contents)")
            out.append("")
            libraries_emitted = True

        hashes = "#" * level
        out.append(f"{hashes} [{display}](#contents)")
        out.append("")

        section_desc = SECTION_DESCRIPTIONS.get(key)
        if section_desc:
            out.append(f"_{section_desc}_")
            out.append("")

        body = _render_section(key, entries, mode, sub_hl, sort_key)
        out.append(body)
        out.append("")

    # Contributing
    out.append("## [Contributing](#contents)")
    out.append("")
    out.append(
        "Contributions are very welcome! Please read the "
        "[contribution guidelines]"
        "(https://github.com/jslee02/awesome-robotics-libraries/blob/main/CONTRIBUTING.md)"
        " first. Also, please feel free to report any error."
    )
    out.append("")

    # Star History
    out.append("## [Star History](#contents)")
    out.append("")
    out.append(
        "[![Star History Chart]"
        "(https://api.star-history.com/svg?repos=jslee02/awesome-robotics-libraries&type=Date)]"
        "(https://star-history.com/#jslee02/awesome-robotics-libraries)"
    )
    out.append("")

    # License
    out.append("## [License](#contents)")
    out.append("")
    out.append(
        "[![CC0](https://licensebuttons.net/p/zero/1.0/88x31.png)]"
        "(http://creativecommons.org/publicdomain/zero/1.0/)"
    )
    out.append("")

    return "\n".join(out)


def main():
    parser = argparse.ArgumentParser(description="Generate README.md from YAML data")
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        help="Output file path (default: README.md in repo root)",
    )
    parser.add_argument(
        "--sort",
        choices=["name", "stars", "last_commit", "none"],
        default="name",
        help="Sort entries within each section (default: name)",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    data_dir = repo_root / "data"

    if not data_dir.exists():
        print(f"ERROR: {data_dir} not found", file=sys.stderr)
        sys.exit(1)

    readme = generate(data_dir, sort_key=args.sort)

    output_path = Path(args.output) if args.output else repo_root / "README.md"
    output_path.write_text(readme, encoding="utf-8")
    print(f"Wrote {output_path}")


if __name__ == "__main__":
    main()
