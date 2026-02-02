#!/usr/bin/env python3
"""One-time script: parse existing README.md into YAML data files.

This is NOT part of the regular workflow — it bootstraps the data/ directory
from the hand-written README.  After the initial conversion, data/*.yaml
becomes the source of truth and this script can be deleted.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# We avoid a PyYAML dependency for the *parser* by emitting YAML manually.
# The output is simple enough that hand-written YAML is perfectly safe.
# ---------------------------------------------------------------------------


def _yaml_str(s: str) -> str:
    """Wrap a string in quotes if it contains special YAML characters."""
    if not s:
        return '""'
    needs_quote = (
        any(c in s for c in ":{}[]&*?|>!%@`,#")
        or s.startswith("- ")
        or s.startswith(" ")
    )
    if needs_quote or s.strip() != s:
        escaped = s.replace("\\", "\\\\").replace('"', '\\"')
        return f'"{escaped}"'
    return s


def _yaml_list_inline(items: list[str]) -> str:
    return "[" + ", ".join(_yaml_str(i) for i in items) + "]"


# ── README structure knowledge ────────────────────────────────────────────

# Maps section heading text → output YAML filename (without .yaml)
SECTION_MAP = {
    "Simulators": "simulators",
    "Dynamics Simulation": "dynamics-simulation",
    "Inverse Kinematics": "inverse-kinematics",
    "Machine Learning": "machine-learning",
    "Motion Planning and Control": "motion-planning",
    "Optimization": "optimization",
    "Robot Modeling": "robot-modeling",
    "Robot Platform": "robot-platform",
    "Reinforcement Learning for Robotics": "reinforcement-learning",
    "SLAM": "slam",
    "Vision": "vision",
    "Fluid": "fluid",
    "Grasping": "grasping",
    "Humanoid Robotics": "humanoid-robotics",
    "Multiphysics": "multiphysics",
    "Math": "math",
    "ETC": "etc",
    "Other Awesome Lists": "other-awesome-lists",
}

# ── Regex helpers ──────────────────────────────────────────────────────────

# Matches:  [Name](url) or just [Name]
RE_LINK = re.compile(r"\[([^\]]+)\]\(([^)]+)\)")
# Matches: [[github](url) ![alt](badge)]  or  [[bitbucket](url)]
RE_REPO = re.compile(r"\[\[?(github|bitbucket|gitlab)\]?\(([^)]+)\)")
# Matches: ![alt](https://img.shields.io/github/stars/OWNER/REPO...)
RE_STARS = re.compile(r"github/stars/([^.]+?)\.svg")


def _extract_github(text: str) -> str | None:
    """Extract owner/repo from a shields.io badge or github URL."""
    m = RE_STARS.search(text)
    if m:
        return m.group(1)
    m = re.search(r"github\.com/([^/]+/[^/)\s]+)", text)
    if m:
        val = m.group(1).rstrip("/").rstrip(")")
        # Remove trailing anchors or query params
        val = val.split("#")[0].split("?")[0]
        return val
    return None


def _extract_bitbucket(text: str) -> str | None:
    m = re.search(r"bitbucket\.org/([^/]+/[^/)\s]+)", text)
    if m:
        return m.group(1).rstrip("/").rstrip(")")
    return None


def _extract_gitlab(text: str) -> str | None:
    m = re.search(r"gitlab\.com/([^/]+/[^/)\s]+)", text)
    if m:
        return m.group(1).rstrip("/").rstrip(")")
    return None


def _clean_desc(text: str) -> str:
    """Remove markdown links, badges, trailing whitespace from a description."""
    # Remove badge images
    text = re.sub(r"!\[[^\]]*\]\([^)]*\)", "", text)
    # Remove link syntax but keep text
    text = re.sub(r"\[([^\]]+)\]\([^)]+\)", r"\1", text)
    # Remove leftover brackets
    text = text.strip().strip("-").strip()
    return text


# ── Dynamics table parser ──────────────────────────────────────────────────


def parse_dynamics_table(lines: list[str]) -> list[dict]:
    """Parse the dynamics simulation markdown table."""
    entries = []
    for line in lines:
        line = line.strip()
        if not line.startswith("|"):
            continue
        cells = [c.strip() for c in line.split("|")]
        if cells and cells[0] == "":
            cells = cells[1:]
        if cells and cells[-1] == "":
            cells = cells[:-1]
        if len(cells) < 7:
            continue
        name_cell = cells[0]
        if (
            name_cell.startswith(":")
            or name_cell.startswith("-")
            or name_cell == "Name"
        ):
            continue

        # Parse name + url
        link_m = RE_LINK.search(name_cell)
        if link_m:
            name = link_m.group(1)
            url = link_m.group(2)
        else:
            name = name_cell.strip()
            url = None

        models_raw = cells[1].strip() if len(cells) > 1 else ""
        features_raw = cells[2].strip() if len(cells) > 2 else ""
        languages_raw = cells[3].strip() if len(cells) > 3 else ""
        license_raw = cells[4].strip() if len(cells) > 4 else ""
        code_cell = cells[5].strip() if len(cells) > 5 else ""
        # popularity cell is just the badge — we derive github from it
        pop_cell = cells[6].strip() if len(cells) > 6 else ""

        entry: dict = {"name": name}
        if url:
            entry["url"] = url

        github = _extract_github(pop_cell) or _extract_github(code_cell)
        if github:
            entry["github"] = github

        bitbucket = _extract_bitbucket(code_cell)
        if bitbucket and not github:
            entry["bitbucket"] = bitbucket

        gitlab = _extract_gitlab(code_cell)
        if gitlab and not github:
            entry["gitlab"] = gitlab

        # If code_cell has a non-github/bitbucket/gitlab link
        if not github and not bitbucket and not gitlab:
            link_m2 = RE_LINK.search(code_cell)
            if link_m2:
                entry["code_url"] = link_m2.group(2)

        models = [
            m.strip()
            for m in models_raw.replace("—", "").split(",")
            if m.strip() and m.strip() != "-"
        ]
        if models:
            entry["models"] = models

        features = [
            f.strip()
            for f in features_raw.replace("—", "").split(",")
            if f.strip() and f.strip() != "-"
        ]
        if features:
            entry["features"] = features

        languages = [l.strip() for l in languages_raw.split(",") if l.strip()]
        if languages:
            entry["languages"] = languages

        if license_raw and license_raw != "—":
            # Extract from link if present
            lm = RE_LINK.search(license_raw)
            entry["license"] = lm.group(1) if lm else license_raw

        entries.append(entry)
    return entries


# ── Bullet list parser ─────────────────────────────────────────────────────


def parse_bullet_entry(line: str) -> dict | None:
    """Parse a single bullet-list entry line."""
    line = line.strip()
    if not line.startswith("*") and not line.startswith("-"):
        return None
    line = line.lstrip("*- ").strip()
    if not line:
        return None

    # Check if line starts with a markdown link [Name](url) vs plain text.
    # Plain text entries like "AirSim (archived) - ..." have no name link;
    # RE_LINK.search would wrongly match [[github](...)] badges instead.
    if line.startswith("[") and not line.startswith("[["):
        m = RE_LINK.match(line)
        if m:
            name = m.group(1)
            url = m.group(2)
        else:
            name = line.split(" - ")[0].split(" [[")[0].strip()
            name = re.sub(r"\(archived\)", "", name).strip()
            url = None
    else:
        # Plain text name (no homepage link)
        name = line.split(" - ")[0].split(" [[")[0].strip()
        name = re.sub(r"\(archived\)", "", name).strip()
        url = None

    entry: dict = {"name": name}
    if url:
        entry["url"] = url

    # Check for archived flag
    if "(archived)" in line:
        entry["archived"] = True
        line = line.replace("(archived)", "").strip()

    # Extract github/bitbucket/gitlab
    github = _extract_github(line)
    if github:
        entry["github"] = github
    else:
        bb = _extract_bitbucket(line)
        if bb:
            entry["bitbucket"] = bb
        gl = _extract_gitlab(line)
        if gl:
            entry["gitlab"] = gl

    # Extract description (text after " - " separator)
    desc_m = re.search(r"\]\([^)]+\)\s*-\s*(.+?)(?:\s*\[\[|\s*$)", line)
    if not desc_m:
        # Fallback for plain-text name entries: "Name - Description [[github..."
        desc_m = re.search(r"^[^[\]]+?\s-\s(.+?)(?:\s*\[\[|\s*$)", line)
    if desc_m:
        desc = _clean_desc(desc_m.group(1))
        if desc:
            entry["description"] = desc

    return entry


# ── Simple table parser (2 columns: Name | Stars) ─────────────────────────


def parse_simple_table(lines: list[str]) -> list[dict]:
    """Parse a simple 2-column Name|Stars table."""
    entries = []
    for line in lines:
        line = line.strip()
        if not line.startswith("|"):
            continue
        cells = [c.strip() for c in line.split("|")]
        cells = [c for c in cells if c != ""]
        if len(cells) < 1:
            continue
        name_cell = cells[0]
        if name_cell.startswith("-") or name_cell == "Name":
            continue

        m = RE_LINK.search(name_cell)
        if not m:
            continue
        name = m.group(1)
        url = m.group(2)

        entry: dict = {"name": name, "url": url}

        # Extract github from stars badge in second column
        full_line = line
        github = _extract_github(full_line)
        if github:
            entry["github"] = github

        entries.append(entry)
    return entries


# ── Main parsing logic ─────────────────────────────────────────────────────

# Sub-sections within a category (stored as subsection metadata)
SUB_SECTIONS = {
    "Simulators": ["Free or Open Source", "Commercial", "Cloud"],
    "Motion Planning and Control": [
        None,
        "Motion Optimizer",
        "Nearest Neighbor",
        "3D Mapping",
    ],
    "Robot Modeling": [
        "Robot Model Description Format",
        "Utility to Build Robot Models",
    ],
    "SLAM": [None, "SLAM Dataset"],
}


def parse_readme(readme_path: Path) -> dict[str, list[dict]]:
    """Parse the full README into {section_key: [entries]}."""
    text = readme_path.read_text(encoding="utf-8")
    lines = text.split("\n")

    sections: dict[str, list[dict]] = {}
    current_section: str | None = None
    current_subsection: str | None = None
    section_lines: list[str] = []
    is_dynamics_table = False

    def flush():
        nonlocal section_lines, is_dynamics_table
        if current_section and section_lines:
            key = SECTION_MAP.get(current_section)
            if key:
                if is_dynamics_table:
                    entries = parse_dynamics_table(section_lines)
                elif any(
                    l.strip().startswith("|") and "Stars" in l
                    for l in section_lines[:5]
                ):
                    entries = parse_simple_table(section_lines)
                else:
                    entries = []
                    for sl in section_lines:
                        e = parse_bullet_entry(sl)
                        if e:
                            if current_subsection:
                                e["_subsection"] = current_subsection
                            entries.append(e)
                if key in sections:
                    sections[key].extend(entries)
                else:
                    sections[key] = entries
        section_lines = []
        is_dynamics_table = False

    for line in lines:
        stripped = line.strip()

        # Detect section headings: ## [Name](#...) or ### [Name](#...)
        heading_m = re.match(r"^#{2,4}\s+\[([^\]]+)\]", stripped)
        if heading_m:
            flush()
            current_section = heading_m.group(1)
            current_subsection = None
            if current_section == "Dynamics Simulation":
                is_dynamics_table = True
            continue

        # Detect sub-section headings: ###### Name
        sub_m = re.match(r"^#{5,6}\s+(.+)", stripped)
        if sub_m:
            flush()
            current_subsection = sub_m.group(1).strip()
            continue

        # Detect sub-section headings: #### Name (used for SLAM Dataset)
        sub_m2 = re.match(r"^####\s+(.+)", stripped)
        if sub_m2 and current_section:
            flush()
            current_subsection = sub_m2.group(1).strip()
            continue

        section_lines.append(line)

    flush()
    return sections


def write_yaml(sections: dict[str, list[dict]], data_dir: Path):
    """Write each section as a YAML file."""
    data_dir.mkdir(exist_ok=True)

    for key, entries in sections.items():
        path = data_dir / f"{key}.yaml"
        with open(path, "w", encoding="utf-8") as f:
            f.write(f"# {key}\n")
            f.write(
                f"# Auto-generated from README.md — do not edit manually after migration\n\n"
            )
            for entry in entries:
                f.write(f"- name: {_yaml_str(entry['name'])}\n")
                for field in [
                    "url",
                    "github",
                    "gitlab",
                    "bitbucket",
                    "code_url",
                    "description",
                    "license",
                ]:
                    if field in entry:
                        f.write(f"  {field}: {_yaml_str(entry[field])}\n")
                if entry.get("archived"):
                    f.write(f"  archived: true\n")
                for list_field in ["languages", "models", "features"]:
                    if list_field in entry and entry[list_field]:
                        f.write(
                            f"  {list_field}: {_yaml_list_inline(entry[list_field])}\n"
                        )
                if "_subsection" in entry:
                    f.write(f"  _subsection: {_yaml_str(entry['_subsection'])}\n")
                f.write("\n")

        print(f"  Wrote {path} ({len(entries)} entries)")


def main():
    repo_root = Path(__file__).resolve().parent.parent
    readme_path = repo_root / "README.md"
    data_dir = repo_root / "data"

    if not readme_path.exists():
        print(f"ERROR: {readme_path} not found", file=sys.stderr)
        sys.exit(1)

    print("Parsing README.md ...")
    sections = parse_readme(readme_path)

    total = sum(len(v) for v in sections.values())
    print(f"Found {total} entries across {len(sections)} sections")

    print(f"\nWriting YAML files to {data_dir}/ ...")
    write_yaml(sections, data_dir)

    print(f"\nDone! {total} entries written to {len(sections)} YAML files.")


if __name__ == "__main__":
    main()
