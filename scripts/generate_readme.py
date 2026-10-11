#!/usr/bin/env python3
"""Generate README.md from data/*.yaml files."""

from __future__ import annotations

import argparse
import sys
from datetime import date
from pathlib import Path

import yaml

# ── Category registry ───────────────────────────────────────────────────────
# (yaml_key, title, scope). The scope line is the category definition shown in
# the README and used by submitters and triage; keep it to one sentence.
# Classify by an entry's main function, never by robot domain.

SECTIONS = [
    (
        "middleware",
        "Middleware & Frameworks",
        "Communication middleware, robot software frameworks, runtime services such as "
        "launch and diagnostics, and agent orchestration for robots.",
    ),
    (
        "hardware",
        "Robot Hardware & Drivers",
        "Open robot platforms and hardware designs, and drivers or firmware for sensors and actuators.",
    ),
    (
        "robot-models",
        "Robot Description & Models",
        "Robot description formats (URDF, SDF, MJCF, USD), their parsers and exporters, "
        "and robot model collections.",
    ),
    (
        "kinematics-dynamics",
        "Kinematics & Dynamics",
        "Forward and inverse kinematics and rigid-body dynamics algorithms, without contact "
        "time-stepping.",
    ),
    (
        "physics-engines",
        "Physics Engines",
        "Embeddable engines that time-step rigid, soft, fluid, or particle dynamics with "
        "contact. See also [Comparisons](COMPARISONS.md).",
    ),
    (
        "simulators",
        "Simulators",
        "Simulation applications with worlds, sensors, rendering, and robot middleware bridges.",
    ),
    (
        "motion-planning",
        "Motion Planning & Navigation",
        "Path planning, trajectory generation and time-parameterization, task and behavior "
        "planning, and navigation stacks.",
    ),
    (
        "control",
        "Control",
        "Model-based control: MPC, trajectory optimization, whole-body and task-space control.",
    ),
    (
        "optimization",
        "Optimization",
        "General-purpose numerical solvers (LP, QP, NLP, least squares, black-box) and "
        "modeling layers.",
    ),
    (
        "math",
        "Math & Geometry",
        "Linear algebra, Lie groups, spatial transforms, and spatial data structures such as "
        "nearest-neighbor search.",
    ),
    (
        "slam",
        "SLAM & State Estimation",
        "SLAM, odometry, factor graphs, filtering and sensor fusion, calibration, and mapping.",
    ),
    (
        "perception",
        "Perception",
        "Image and point-cloud processing, detection, pose estimation and tracking, and visual "
        "servoing.",
    ),
    (
        "grasping",
        "Grasping & Manipulation",
        "Grasp synthesis, detection, and planning, and dexterous manipulation.",
    ),
    (
        "robot-learning",
        "Robot Learning",
        "Robot foundation models (VLA), imitation and reinforcement learning libraries, "
        "learning environments, and benchmarks.",
    ),
    (
        "robot-data",
        "Robot Data & Visualization",
        "Logging, storing, converting, and visualizing robot data, dataset tools, and "
        "teleoperation for data collection.",
    ),
    (
        "other-awesome-lists",
        "Other Awesome Lists",
        "Related curated lists and collections.",
    ),
]

# Subsections rendered after a category's main entries, in this order.
SUBSECTIONS = {
    "physics-engines": ["Soft, Fluid & Multiphysics"],
    "simulators": ["Commercial"],
    "kinematics-dynamics": ["Inverse Kinematics"],
    "slam": ["Mapping"],
    "robot-learning": ["General Machine Learning"],
}

# Parent → children rendered as indented bullets right after the parent.
CHILDREN = {
    "robot-learning": {"Gymnasium": ["gym-dart", "gym-gazebo"]},
    "hardware": {"Linorobot": ["onine"]},
}

ACTIVE_DAYS = 365
INACTIVE_DAYS = 3 * 365
AS_OF_FILE = "as-of.txt"


def load_entries(path: Path) -> list[dict]:
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data if isinstance(data, list) else []


def as_of_date(data_dir: Path) -> date:
    """Reference date for activity badges.

    Read from data/as-of.txt (written by the metadata refresh) so that the
    generated README depends only on the repository contents, not on the day
    the generator runs.
    """
    path = data_dir / AS_OF_FILE
    if path.exists():
        return date.fromisoformat(path.read_text(encoding="utf-8").strip())
    return date.today()


def stars(entry: dict) -> int | None:
    return (entry.get("_meta") or {}).get("stars")


def is_archived(entry: dict) -> bool:
    return bool(entry.get("archived") or (entry.get("_meta") or {}).get("archived"))


def idle_days(entry: dict, as_of: date) -> int | None:
    last_commit = (entry.get("_meta") or {}).get("last_commit")
    if not last_commit:
        return None
    try:
        return (as_of - date.fromisoformat(str(last_commit))).days
    except ValueError:
        return None


def is_inactive(entry: dict, as_of: date) -> bool:
    days = idle_days(entry, as_of)
    return is_archived(entry) or (days is not None and days > INACTIVE_DAYS)


def sort_entries(entries: list[dict], children: dict[str, list[str]]) -> list[tuple[dict, bool]]:
    """Return (entry, is_child) pairs: most stars first, entries without stars
    (not on GitHub) last by name.

    A child follows its parent in CHILDREN order when both are in `entries`;
    otherwise it sorts like any other entry.
    """
    by_name = {e["name"]: e for e in entries}
    nested = {c for parent, kids in children.items() if parent in by_name for c in kids}
    parents = sorted(
        (e for e in entries if e["name"] not in nested),
        key=lambda e: (stars(e) is None, -(stars(e) or 0), e["name"].lower()),
    )
    result = []
    for parent in parents:
        result.append((parent, False))
        result += [(by_name[c], True) for c in children.get(parent["name"], []) if c in by_name]
    return result


def format_stars(count: int) -> str:
    if count >= 1000:
        return f"{count / 1000:.1f}k".replace(".0k", "k")
    return str(count)


def repo_link(entry: dict) -> tuple[str, str] | None:
    """Return (label, url) for the source repository, if any."""
    for key, base in (
        ("github", "https://github.com/"),
        ("gitlab", "https://gitlab.com/"),
        ("bitbucket", "https://bitbucket.org/"),
    ):
        if entry.get(key):
            return key, base + entry[key]
    if entry.get("code_url"):
        return "code", entry["code_url"]
    return None


def render_bullet(entry: dict, as_of: date, indent: str = "", folded: bool = False) -> str:
    parts = [f"{indent}*"]
    days = idle_days(entry, as_of)
    if is_archived(entry):
        parts.append("💀")
    elif not folded and days is not None:
        parts.append("🟢" if days <= ACTIVE_DAYS else "🟡")

    url = entry.get("url")
    parts.append(f"[{entry['name']}]({url})" if url else entry["name"])
    if entry.get("description"):
        parts.append(f"- {entry['description']}")

    repo = repo_link(entry)
    if repo:
        label, repo_url = repo
        count = stars(entry)
        parts.append(f"[⭐ {format_stars(count)}]({repo_url})" if count is not None else f"[[{label}]({repo_url})]")

    last_commit = (entry.get("_meta") or {}).get("last_commit")
    if folded and last_commit:
        parts.append(f"· last commit {str(last_commit)[:4]}")
    return " ".join(parts)


def render_list(entries: list[dict], children: dict[str, list[str]], as_of: date, folded: bool) -> list[str]:
    return [
        render_bullet(e, as_of, indent="  " if is_child else "", folded=folded)
        for e, is_child in sort_entries(entries, children)
    ]


def render_section(key: str, entries: list[dict], as_of: date) -> list[str]:
    children = CHILDREN.get(key, {})
    active = [e for e in entries if not is_inactive(e, as_of)]
    inactive = [e for e in entries if is_inactive(e, as_of)]
    subsections = SUBSECTIONS.get(key, [])

    lines: list[str] = []
    main = [e for e in active if e.get("_subsection") not in subsections]
    if main:
        lines += render_list(main, children, as_of, folded=False) + [""]
    for sub in subsections:
        sub_entries = [e for e in active if e.get("_subsection") == sub]
        if sub_entries:
            lines += [f"### {sub}", ""] + render_list(sub_entries, children, as_of, folded=False) + [""]
    if inactive:
        lines += [
            "<details>",
            f"<summary>Inactive ({len(inactive)}): archived or no commits for 3+ years</summary>",
            "",
        ]
        lines += render_list(inactive, children, as_of, folded=True)
        lines += ["", "</details>", ""]
    return lines


def anchor(title: str) -> str:
    """GitHub heading anchor: lowercase, drop punctuation, spaces to hyphens."""
    kept = "".join(ch for ch in title.lower() if ch.isalnum() or ch in " -")
    return kept.replace(" ", "-")


def generate(data_dir: Path) -> str:
    as_of = as_of_date(data_dir)
    sections = [
        (key, title, scope, load_entries(data_dir / f"{key}.yaml"))
        for key, title, scope in SECTIONS
        if (data_dir / f"{key}.yaml").exists()
    ]

    out = [
        "# Awesome Robotics Libraries",
        "",
        "[![Awesome](https://awesome.re/badge.svg)](https://awesome.re)",
        "",
        "A curated list of robotics libraries and simulators.",
        "",
        "## Contents",
    ]
    out += [f"* [{title}](#{anchor(title)})" for _, title, _, entries in sections if entries]
    out += [
        "* [Contributing](#contributing)",
        "",
        "> **Legend**: 🟢 commit within 1 yr · 🟡 commit within 3 yr · 💀 archived · "
        f"⭐ GitHub stars as of {as_of.isoformat()}. Entries are ordered by stars; inactive "
        "entries (archived, or no commits for 3+ years) are folded at the end of each category.",
        "",
    ]

    for key, title, scope, entries in sections:
        if not entries:
            continue
        out += [f"## [{title}](#contents)", "", f"_{scope}_", ""]
        out += render_section(key, entries, as_of)

    out += [
        "## [Contributing](#contents)",
        "",
        "Contributions are very welcome! Please read the "
        "[contribution guidelines](https://github.com/jslee02/awesome-robotics-libraries/blob/main/CONTRIBUTING.md)"
        " first. Also, please feel free to report any error.",
        "",
        "## [Star History](#contents)",
        "",
        '<a href="https://www.star-history.com/?repos=jslee02%2Fawesome-robotics-libraries">\n'
        "  <picture>\n"
        '    <source media="(prefers-color-scheme: dark)" '
        'srcset="https://api.star-history.com/chart?repos=jslee02/awesome-robotics-libraries&type=date&theme=dark&legend=top-left" />\n'
        '    <source media="(prefers-color-scheme: light)" '
        'srcset="https://api.star-history.com/chart?repos=jslee02/awesome-robotics-libraries&type=date&legend=top-left" />\n'
        '    <img alt="Star History Chart" '
        'src="https://api.star-history.com/chart?repos=jslee02/awesome-robotics-libraries&type=date&legend=top-left" />\n'
        "  </picture>\n"
        "</a>",
        "",
        "## [License](#contents)",
        "",
        "[![CC0](https://licensebuttons.net/p/zero/1.0/88x31.png)](http://creativecommons.org/publicdomain/zero/1.0/)",
        "",
    ]
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

    output_path = Path(args.output) if args.output else repo_root / "README.md"
    output_path.write_text(generate(data_dir), encoding="utf-8")
    print(f"Wrote {output_path}")


if __name__ == "__main__":
    main()
