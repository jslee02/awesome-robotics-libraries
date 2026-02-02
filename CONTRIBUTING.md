# Contributing

Thank you for improving this list! This guide explains how to add libraries, fix entries, and work with the project.

## How It Works

This repository uses a **YAML-first** workflow:

```
data/*.yaml  -->  scripts/generate_readme.py  -->  README.md
```

- **Source of truth**: YAML files in `data/` (one per section)
- **Schema**: `schema/entry.schema.json` defines valid entry fields
- **Generator**: `scripts/generate_readme.py` builds README.md from YAML
- **Do not edit README.md by hand** — changes will be overwritten on the next generation

## Adding a Library

1. Find the right YAML file in `data/` (e.g., `data/simulators.yaml`, `data/slam.yaml`)

2. Add your entry in alphabetical order within the file:

   ```yaml
   - name: MyLibrary
     url: https://mylibrary.org
     github: owner/repo
     description: One-line description of what it does
   ```

3. Regenerate README.md:

   ```bash
   pixi run generate
   ```

4. Commit both the YAML file and README.md:

   ```bash
   git add data/section-name.yaml README.md
   git commit --signoff -m "content: add MyLibrary to section-name"
   ```

> **Tip**: If you cannot run the generator locally, submit your PR with just the YAML change and note it in the PR description. A maintainer will regenerate README.md for you.

### Entry Fields

| Field | Required | Description |
|-------|----------|-------------|
| `name` | Yes | Library display name |
| `url` | | Project website |
| `github` | | GitHub `owner/repo` (e.g., `dartsim/dart`) |
| `gitlab` | | GitLab path |
| `bitbucket` | | Bitbucket path |
| `description` | | Brief description |
| `license` | | SPDX license identifier (dynamics table only) |
| `languages` | | List of programming languages (dynamics table only) |
| `models` | | Supported models like `rigid`, `soft`, `fluid` (dynamics table only) |
| `features` | | Features like `ik`, `id`, `urdf` (dynamics table only) |
| `archived` | | Set `true` if the project is archived |
| `code_url` | | Direct download link (if no repo hosting) |
| `_subsection` | | Subsection grouping within a section |

### Which YAML File?

| Section | File |
|---------|------|
| Simulators | `data/simulators.yaml` |
| Dynamics Simulation | `data/dynamics-simulation.yaml` |
| Inverse Kinematics | `data/inverse-kinematics.yaml` |
| Machine Learning | `data/machine-learning.yaml` |
| Motion Planning and Control | `data/motion-planning.yaml` |
| Optimization | `data/optimization.yaml` |
| Robot Modeling | `data/robot-modeling.yaml` |
| Robot Platform | `data/robot-platform.yaml` |
| Reinforcement Learning for Robotics | `data/reinforcement-learning.yaml` |
| SLAM | `data/slam.yaml` |
| Vision | `data/vision.yaml` |
| Fluid | `data/fluid.yaml` |
| Grasping | `data/grasping.yaml` |
| Humanoid Robotics | `data/humanoid-robotics.yaml` |
| Multiphysics | `data/multiphysics.yaml` |
| Math | `data/math.yaml` |
| ETC | `data/etc.yaml` |
| Other Awesome Lists | `data/other-awesome-lists.yaml` |

## Editing Existing Entries

Edit the corresponding field in the relevant `data/*.yaml` file, then regenerate README.md.

Common edits:
- **Fix a URL**: update `url` or `github`
- **Update description**: update `description`
- **Mark as archived**: add `archived: true`
- **Remove an entry**: delete the entire entry block

## Editing Non-Data Content

Content that is not per-entry data (section headings, table of contents, dynamics table legend, contributing/license text) is defined in `scripts/generate_readme.py`. Edit the script directly for these changes.

## Local Development

### Prerequisites

- Python 3.10+

### Validate entries

```bash
pip install pyyaml jsonschema
python3 scripts/validate_entries.py
```

### Regenerate README.md

```bash
pip install pyyaml
python3 scripts/generate_readme.py
```

### Check links (optional)

Install [lychee](https://lychee.cli.rs/) and run:

```bash
lychee --config .lychee.toml '**/*.md'
```

## What CI Checks

Every pull request that touches `data/`, `schema/`, or `scripts/` runs these checks:

| Check | Description |
|-------|-------------|
| **Validate Data** | YAML entries pass JSON schema validation |
| **README Freshness** | README.md matches the generated output from YAML |
| **Check Links** | All URLs in markdown files are reachable |
| **DCO** | Commits are signed off (`git commit --signoff`) |

## Guidelines

- One library per pull request (when possible)
- Alphabetical order within each section
- Keep descriptions short and descriptive
- Sign your commits: `git commit --signoff` (required by DCO check)
- Check your spelling and grammar

## For Maintainers

### Regenerating README for a contributor

If a contributor submits a YAML-only PR without regenerating README.md:

```bash
gh pr checkout <PR-NUMBER>
pip install pyyaml
python3 scripts/generate_readme.py
git add README.md
git commit --signoff -m "docs: regenerate README.md"
git push
```

### Automated workflows

| Workflow | Schedule | Description |
|----------|----------|-------------|
| Check Links | Weekly (Mon 8am UTC) | Checks for broken links; auto-creates fix PRs |
| Validate Data | On PR | Validates YAML schema and README freshness |
