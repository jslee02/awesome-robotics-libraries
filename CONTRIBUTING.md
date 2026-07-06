# Contributing

Thank you for improving this list! This guide explains how to suggest resources, fix entries, and work with the project.

> [!TIP]
> **Want to add something to the list?** The easiest way is to [open a suggestion issue](https://github.com/jslee02/awesome-robotics-libraries/issues/new?template=suggest-resource.yml). Our automation evaluates GitHub-hosted projects automatically, and once accepted, a PR is created for you — no local setup needed.

> [!CAUTION]
> **Do not edit `README.md` directly.** It is auto-generated from YAML data files. Manual edits will be overwritten, and PRs that only modify `README.md` will be rejected by CI.

## How It Works

This repository uses a **YAML-first** workflow:

```
data/*.yaml  →  scripts/generate_readme.py  →  README.md
```

| Component | Role |
|-----------|------|
| `data/*.yaml` | **Source of truth** — one file per section |
| `schema/entry.schema.json` | Defines valid entry fields |
| `scripts/generate_readme.py` | Generates `README.md` from YAML |

## Ways to Contribute

| What you want to do | How |
|---------------------|-----|
| **Suggest a new resource** | [Open a suggestion issue](https://github.com/jslee02/awesome-robotics-libraries/issues/new?template=suggest-resource.yml) |
| **Fix a URL or description** | Edit the relevant `data/*.yaml` file and [open a PR](#editing-existing-entries) |
| **Report broken links or errors** | [Open a report issue](https://github.com/jslee02/awesome-robotics-libraries/issues/new?template=report-issue.yml) |
| **Change section headings or layout** | Edit `scripts/generate_readme.py` and open a PR |

## Suggesting a New Resource

New resources are accepted through the issue-based suggestion workflow. Direct pull requests that add a new resource entry are closed automatically and redirected here so every suggestion goes through the same evaluation and auto-PR path.

1. Go to **[Suggest a Resource](https://github.com/jslee02/awesome-robotics-libraries/issues/new?template=suggest-resource.yml)**
2. Fill in the name, URL, category, and description
3. Submit — automation handles the rest:
   - GitHub-hosted projects are evaluated against [inclusion criteria](#inclusion--exclusion-criteria) automatically
   - Non-GitHub resources are flagged for manual maintainer review
   - Once accepted, a PR with the YAML entry, metadata, and regenerated README is created automatically
   - A maintainer reviews and merges

No local setup or code changes needed.

### Entry Fields

| Field | Required | Description |
|-------|----------|-------------|
| `name` | Yes | Display name |
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

Every pull request runs applicable checks automatically:

| Check | Triggers On | Description |
|-------|-------------|-------------|
| **Validate Data** | `data/**`, `schema/**`, `scripts/**` | YAML entries pass JSON schema validation |
| **README Freshness** | `data/**`, `schema/**`, `scripts/**` | README.md matches the generated output from YAML |
| **Audit Direct Resource PRs** | `data/**` | Closes direct human PRs that add new resources and redirects them to the suggestion issue form |
| **Protect README** | `README.md` | Rejects PRs that edit README.md without corresponding data/script changes |
| **Check Links** | `**.md` | All URLs in markdown files are reachable |
| **DCO** | All PRs | Commits are signed off (`git commit --signoff`) |

## Guidelines

- One resource per suggestion issue (when possible)
- Alphabetical order within each section
- Keep descriptions short and descriptive
- Sign your commits: `git commit --signoff` (required by DCO check)
- Check your spelling and grammar

## Inclusion & Exclusion Criteria

This is a **curated** list — not every robotics project belongs here. We use objective criteria to keep the list high-quality and fair.

### Acceptance

**Must-have** (all required):

- [ ] Relevant to robotics (simulation, planning, perception, control, dynamics, etc.)
- [ ] Publicly accessible (open source, or well-documented commercial tool)
- [ ] Not a duplicate of an existing entry
- [ ] Has a meaningful description

**Scoring** (need ≥ 3 of 5):

- [ ] **Popularity**: ≥ 50 GitHub stars (or equivalent community adoption for non-GitHub projects)
- [ ] **Activity**: At least one commit within the last 2 years
- [ ] **Documentation**: Has a README with usage examples or API docs
- [ ] **Maturity**: Project is ≥ 6 months old (not premature/experimental)
- [ ] **Uniqueness**: Fills a niche not already covered by existing entries

| Result | Criteria | Action |
|--------|----------|--------|
| ✅ Accept | Meets all must-haves + ≥ 3 scoring points | Add to the main list |
| 🟡 Incubator | Meets all must-haves + 1–2 scoring points | Issue stays open with `incubator` label; re-evaluated when project matures |
| ❌ Reject | Fails any must-have | Issue closed with explanation |

### Removal

An existing entry may be removed if **any** of the following apply:

- Archived **and** superseded by another listed entry (e.g., nphysics → Rapier)
- No commits in 5+ years **and** < 100 stars **and** no historical significance
- URL permanently broken (after link-check CI has flagged it for 3+ months)

### Exceptions

- **Historical significance**: Foundational libraries (e.g., ODE, Simbody) are kept even if inactive — they remain important references.
- **Commercial tools**: Accepted if widely used in the robotics community, even without a public repo.

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
| Refresh Metadata | Weekly (Mon 8am UTC) | Updates GitHub metadata, regenerates README, validates, and creates a maintenance PR |
| Stale Author-Action Issues | Daily | Marks suggestion issues stale after 14 days waiting for author input, then closes after another 14 days |
| Validate Data | On PR | Validates YAML schema and README freshness |
| Audit Direct Resource PRs | On PR | Closes direct human PRs that add new resources and redirects them to the suggestion issue form |
| Protect README | On PR | Blocks direct README.md edits without data changes |

For metadata PRs to trigger normal pull-request CI without manual approval,
configure a repository secret named `CREATE_PR_TOKEN` with a fine-grained bot
token or GitHub App token that has `contents: write` and `pull-requests: write`.
The workflow falls back to `GITHUB_TOKEN` when that secret is not present.
