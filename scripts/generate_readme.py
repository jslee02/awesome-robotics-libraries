#!/usr/bin/env python3
"""Generate README.md from data/*.yaml files."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

# ── Section registry ────────────────────────────────────────────────────────
# (yaml_key, display_name, heading_level, render_mode, subsection_heading_level)
# render_mode: "bullet", "dynamics_table", "simple_table"
# subsection_heading_level: 6 means ######, 4 means ####, None means no subsections

SECTIONS = [
    ("simulators", "Simulators", 2, "bullet", 6),
    # -- "Libraries" divider inserted here --
    ("dynamics-simulation", "Dynamics Simulation", 3, "dynamics_table", None),
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
    "dynamics-simulation": "Physics engines and rigid/soft body dynamics libraries for robotics.",
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


def _stars_badge(github: str) -> str:
    return (
        f"https://img.shields.io/github/stars/"
        f"{github}.svg?style=flat&label=Star&maxAge=86400"
    )


# ── Bullet rendering ───────────────────────────────────────────────────────


def _render_bullet(entry: dict, indent: str = "") -> str:
    name = entry["name"]
    url = entry.get("url")
    github = entry.get("github")
    bitbucket = entry.get("bitbucket")
    gitlab = entry.get("gitlab")
    desc = entry.get("description", "")
    archived = entry.get("archived", False)

    parts = [f"{indent}*"]

    # Name part
    if archived:
        if url:
            parts.append(f"[{name}]({url}) (archived)")
        else:
            parts.append(f"{name} (archived)")
    elif url:
        parts.append(f"[{name}]({url})")
    else:
        parts.append(name)

    # Description
    if desc:
        parts.append(f"- {desc}")

    # Repo badge
    if github:
        badge = _stars_badge(github)
        parts.append(f"[[github](https://github.com/{github}) ![{name}]({badge})]")
    elif bitbucket:
        parts.append(f"[[bitbucket](https://bitbucket.org/{bitbucket})]")
    elif gitlab:
        parts.append(f"[[gitlab](https://gitlab.com/{gitlab})]")

    return " ".join(parts)


# ── Dynamics table rendering ────────────────────────────────────────────────

DYNAMICS_HEADER = """\
> :warning: The following table is not complete. Please feel free to report if you find something incorrect or missing.

| Name | Models | Features | Languages | Licenses | Code | Popularity |
|:----:| ------ | -------- | --------- | -------- | ---- | ---------- |"""


def _render_dynamics_row(entry: dict) -> str:
    name = entry["name"]
    url = entry.get("url")
    github = entry.get("github")
    bitbucket = entry.get("bitbucket")
    gitlab = entry.get("gitlab")
    code_url = entry.get("code_url")

    name_cell = f"[{name}]({url})" if url else name
    models = ", ".join(entry.get("models", [])) or "—"
    features = ", ".join(entry.get("features", [])) or "—"
    languages = ", ".join(entry.get("languages", [])) or "—"
    license_val = entry.get("license", "—") or "—"

    # Code cell (show all available repo links)
    code_parts = []
    if bitbucket:
        code_parts.append(f"[bitbucket](https://bitbucket.org/{bitbucket})")
    if github:
        code_parts.append(f"[github](https://github.com/{github})")
    if gitlab:
        code_parts.append(f"[gitlab](https://gitlab.com/{gitlab})")
    if not code_parts and code_url:
        code_parts.append(f"[download]({code_url})")
    code_cell = ", ".join(code_parts)

    # Popularity cell
    if github:
        badge = _stars_badge(github)
        pop_cell = f"![{name}]({badge})"
    else:
        pop_cell = ""

    # Handle special license link cases
    if license_val == "custom" and github:
        license_cell = (
            f"[custom](https://github.com/{github}/blob/"
            f"a9e7673569997f35c0bc7eb5d11bc4fa188e863c/LICENSE.md)"
        )
    else:
        license_cell = license_val

    return (
        f"| {name_cell} | {models} | {features} "
        f"| {languages} | {license_cell} | {code_cell} | {pop_cell} |"
    )


DYNAMICS_LEGEND = """\

For simplicity, shortened names are used to represent the supported models and features as

* Supported Models
  * rigid: rigid bodies
  * soft: soft bodies
  * aero: aerodynamics
  * granular: granular materials (like sand)
  * fluid: fluid dynamics
  * vehicles
  * uav: unmanned aerial vehicles (like drones)
  * medical
  * molecules
  * parallel: parallel mechanism (like Stewart platform)

* Features on Simulation, Analysis, Planning, Control Design
  * dm: [discrete mechanics](https://www.cambridge.org/core/journals/acta-numerica/article/div-classtitlediscrete-mechanics-and-variational-integratorsdiv/C8F45478A9290DEC24E63BB7FBE3CEB5)
  * ik: [inverse kinematics](https://en.wikipedia.org/wiki/Inverse_kinematics) solvers (please find IK specialized packages in [this list](#inverse-kinematics))
  * id: [inverse dynamics](https://en.wikipedia.org/wiki/Inverse_dynamics)
  * slam: [simultaneous localization and mapping](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)
  * trj-opt: trajectory optimization
  * plan: motion planning algorithms
  * cv: computer vision
  * urdf: [urdf](http://wiki.ros.org/urdf) parser
  * sdf: [sdf](http://sdformat.org/) parser"""


# ── Simple table rendering ──────────────────────────────────────────────────


def _render_simple_table(entries: list[dict]) -> str:
    lines = ["| Name | Stars |", "|------|-------|"]
    for entry in entries:
        name = entry["name"]
        url = entry.get("url", "")
        github = entry.get("github")
        if not url and github:
            url = f"https://github.com/{github}"
        name_cell = f"[{name}]({url})" if url else name
        if github:
            badge = _stars_badge(github)
            stars_cell = f"[![GitHub stars]({badge})](https://github.com/{github})"
        else:
            stars_cell = ""
        lines.append(f"| {name_cell} | {stars_cell} |")
    return "\n".join(lines)


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
    key: str, entries: list[dict], mode: str, sub_heading_level: int | None
) -> str:
    lines: list[str] = []

    if mode == "dynamics_table":
        lines.append(DYNAMICS_HEADER)
        for entry in entries:
            lines.append(_render_dynamics_row(entry))
        lines.append(DYNAMICS_LEGEND)
        return "\n".join(lines)

    if mode == "simple_table":
        return _render_simple_table(entries)

    if mode == "other_awesome":
        return _render_other_awesome(entries)

    # bullet mode
    indent_all = key in INDENT_ALL
    base_indent = "  " if indent_all else ""
    children_map = CHILDREN.get(key, {})

    # Build set of child names for skip logic
    child_names = set()
    for kids in children_map.values():
        child_names.update(kids)

    # Group by subsection if applicable
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
                    # Render children of this header
                    kids = children_map.get(entry["name"], [])
                    for sub_entry in sub_entries:
                        if sub_entry["name"] in kids:
                            lines.append(_render_bullet(sub_entry, indent="  "))
                    continue

                lines.append(_render_bullet(entry, indent=base_indent))
                # Render children
                kids = children_map.get(entry["name"], [])
                for kid_name in kids:
                    for sub_entry in sub_entries:
                        if sub_entry["name"] == kid_name:
                            lines.append(_render_bullet(sub_entry, indent="  "))

            first_sub = False
    else:
        for entry in entries:
            if entry["name"] in child_names:
                continue
            lines.append(_render_bullet(entry, indent=base_indent))
            kids = children_map.get(entry["name"], [])
            for kid_name in kids:
                for sub_entry in entries:
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


def generate(data_dir: Path) -> str:
    out: list[str] = []

    out.append("# Awesome Robotics Libraries")
    out.append("")
    out.append("[![Awesome](https://awesome.re/badge.svg)](https://awesome.re)")
    out.append("")
    out.append("A curated list of robotics simulators and libraries.")
    out.append("")
    out.append(TOC)
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

        body = _render_section(key, entries, mode, sub_hl)
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
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    data_dir = repo_root / "data"

    if not data_dir.exists():
        print(f"ERROR: {data_dir} not found", file=sys.stderr)
        sys.exit(1)

    readme = generate(data_dir)

    output_path = Path(args.output) if args.output else repo_root / "README.md"
    output_path.write_text(readme, encoding="utf-8")
    print(f"Wrote {output_path}")


if __name__ == "__main__":
    main()
