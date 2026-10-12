# Contributing

Thank you for improving this list! This guide explains how resources get in,
how to fix existing entries, and how the automation works.

> [!TIP]
> **Want to add something?** [Open a suggestion](https://github.com/jslee02/awesome-robotics-libraries/issues/new?template=suggest-resource.yml).
> Automation checks it against the bar below and, if it passes, opens the PR for you. No local setup needed.

> [!CAUTION]
> **Do not edit `README.md` directly.** It is generated from `data/*.yaml`.
> Direct README edits are closed automatically.

## Suggesting a Resource

- Use the [suggestion form](https://github.com/jslee02/awesome-robotics-libraries/issues/new?template=suggest-resource.yml), one resource per issue.
- Keep **one open suggestion at a time**. A second one is closed until the first is resolved.
- Suggestions must be **written or reviewed by a person**. Use of AI tools is fine; unreviewed agent output is not.
- **Disclose your affiliation.** Suggesting your own project is welcome; the form just asks.
- Direct pull requests that add a resource are closed and redirected to the form, so every suggestion goes through the same checks.
- Maintainers' own suggestions and maintainer picks skip the confirmation, affiliation, and one-at-a-time rules.

## Acceptance Bar

A resource must be **in scope**: robotics software (a library, simulator, framework, or
tool) that fits one of the [categories](#categories), or a curated list of such resources.

GitHub-hosted projects must then pass these gates, all checked automatically from GitHub data:

| Gate | Regular | Fast lane |
|------|---------|-----------|
| Repository age | 6 months or older | none |
| Stars | 100 or more | 1,000 or more |
| Outside people | 3 or more distinct people who opened issues or PRs and are not owners, members, collaborators, or bots | same |
| Activity | a default-branch commit within 12 months | same |
| Basics | not archived, has a license, not already listed | same |

When a suggestion passes, a maintainer reviews the generated PR for scope and category.

- **Not on GitHub** (GitLab, commercial tools): reviewed by a maintainer, who looks for
  wide use in the robotics community. Fill in **Evidence of use** (independent users or
  customers, citations, third-party tutorials); without it, the bot asks for it first.
- **Maintainer picks**: a maintainer may add a resource outside these gates and says why in the PR. This is rare.

**Why these signals:** AI tools make a polished README, docs, tests, and steady commits
cheap to produce, so they no longer show that a project is used. Stars can be bought, and
since 2026-06-30 GitHub no longer shows outsiders who starred a repository. Time and real
outside users are hard to fake, so the bar rests on them.

## What Happens to a Suggestion

| Outcome | What the bot does | Label |
|---------|-------------------|-------|
| Passes the bar | Opens a PR with the entry. Merging it accepts the suggestion; closing it (with a reason) declines it. | `needs-review` |
| Passes, but the category is unclear (or "Not sure"), or the outside-people count is inconclusive | Waits for a maintainer, who sets the category or checks the count | `needs-review` |
| Below the bar, but at least 20 stars and 1 outside person, with a license and a commit within 12 months | Keeps the issue open, re-checks it monthly for up to 12 months, updates one status comment, and opens the PR when it passes | `monitoring` |
| Below that floor | Closes the issue, listing what is missing. Resubmit any time later. | |
| Missing information, repository not found, no human confirmation or affiliation, or agent-tagged | Asks you for an update. Marked stale after 14 days without one, closed after 14 more. | `question` |
| Already listed, or already suggested in an open issue | Closes with a pointer to the entry or the issue | `duplicate` |
| Archived repository, or a second open suggestion from the same person | Closes with the reason | |

## Categories

Each entry goes in one category, chosen by **what the software mainly does**, never by robot
type (humanoid, drone, car). Tie-breaks:

- You import it and step it in your own loop: **Physics Engines**. You launch a world: **Simulators**.
- It solves a model-based control problem (MPC, trajectory optimization): **Control**. It plans collision-free paths: **Motion Planning & Navigation**.
- It ships tasks, benchmarks, or policies for learning: **Robot Learning**. It records, stores, or visualizes data: **Robot Data & Visualization**.

| Category | Scope | File |
|----------|-------|------|
| Middleware & Frameworks | Communication middleware, robot software frameworks, runtime services such as launch and diagnostics, and agent orchestration for robots | `data/middleware.yaml` |
| Robot Hardware & Drivers | Open robot platforms and hardware designs, and drivers or firmware for sensors and actuators | `data/hardware.yaml` |
| Robot Description & Models | Robot description formats (URDF, SDF, MJCF, USD), their parsers and exporters, and robot model collections | `data/robot-models.yaml` |
| Kinematics & Dynamics | Forward and inverse kinematics and rigid-body dynamics algorithms, without contact time-stepping | `data/kinematics-dynamics.yaml` |
| Physics Engines | Embeddable engines that time-step rigid, soft, fluid, or particle dynamics with contact | `data/physics-engines.yaml` |
| Simulators | Simulation applications with worlds, sensors, rendering, and robot middleware bridges | `data/simulators.yaml` |
| Motion Planning & Navigation | Path planning, trajectory generation and time-parameterization, task and behavior planning, and navigation stacks | `data/motion-planning.yaml` |
| Control | Model-based control: MPC, trajectory optimization, whole-body and task-space control | `data/control.yaml` |
| Optimization | General-purpose numerical solvers (LP, QP, NLP, least squares, black-box) and modeling layers | `data/optimization.yaml` |
| Math & Geometry | Linear algebra, Lie groups, spatial transforms, and spatial data structures such as nearest-neighbor search | `data/math.yaml` |
| SLAM & State Estimation | SLAM, odometry, factor graphs, filtering and sensor fusion, calibration, and mapping | `data/slam.yaml` |
| Perception | Image and point-cloud processing, detection, pose estimation and tracking, and visual servoing | `data/perception.yaml` |
| Grasping & Manipulation | Grasp synthesis, detection, and planning, and dexterous manipulation | `data/grasping.yaml` |
| Robot Learning | Robot foundation models (VLA), imitation and reinforcement learning libraries, learning environments, and benchmarks | `data/robot-learning.yaml` |
| Robot Data & Visualization | Logging, storing, converting, and visualizing robot data, dataset tools, and teleoperation for data collection | `data/robot-data.yaml` |
| Other Awesome Lists | Related curated lists and collections | `data/other-awesome-lists.yaml` |

## Entry Conventions

- **Name**: the project's own name, spelled as the project spells it.
- **Description**: one neutral sentence, 100 characters or fewer, ending with a period. Say
  what it is, not why it is great; don't repeat the name.
- **Order**: the README orders each category by stars; projects not on GitHub come last.
  Keep `data/*.yaml` files alphabetical by name.
- **Inactive entries**: archived projects and projects without a default-branch commit for
  3+ years are folded into a collapsed block at the end of their category, automatically.
  They are kept as references and return when activity resumes.
- **Dead entries**: a GitHub repository that is gone on two weekly metadata refreshes, a
  week apart, is removed in the second refresh PR (the first one, which marks it, must be merged). Broken links elsewhere are reported in a Link Checker issue.

### Entry Fields

| Field | Required | Description |
|-------|----------|-------------|
| `name` | Yes | Display name |
| `url` | | Project website |
| `github` | | GitHub `owner/repo` (e.g., `dartsim/dart`) |
| `gitlab` | | GitLab path |
| `bitbucket` | | Bitbucket path |
| `code_url` | | Direct download link (if no repo hosting) |
| `description` | | One-line description |
| `archived` | | Set `true` if a project not on GitHub is discontinued |
| `license`, `languages`, `models`, `features` | | Physics-engine comparison details |
| `_subsection` | | Subsection within a category |
| `_meta` | | Filled in by automation (GitHub repository ID, stars, last commit, archived, license, language) |

## Fixing Existing Entries

Edit the entry in its `data/*.yaml` file and open a PR. Common fixes:
a URL (`url`, `github`), the `description`, or the category (move the entry to another file).
CI regenerates and checks the README; if you can't run the generator locally, say so in the
PR and a maintainer will regenerate it.

## Local Development

Requires Python 3.10+.

```bash
pip install pyyaml jsonschema
python3 scripts/validate_entries.py   # schema check
python3 scripts/generate_readme.py    # regenerate README.md
```

Optional link check with [lychee](https://lychee.cli.rs/): `lychee --config .lychee.toml '**/*.md'`

## What CI Checks

| Check | Runs on | Description |
|-------|---------|-------------|
| Validate Data | PRs touching `data/`, `schema/`, `scripts/` | Schema validation and README freshness |
| Audit Direct Resource PRs | PRs touching `data/` | Closes human PRs that add a resource and points to the form |
| Protect README | PRs touching `README.md` | Closes README-only PRs from outside contributors |
| Check Links | `**.md` | Fails on definitively broken links |
| DCO | All PRs | Commits must be signed off (`git commit --signoff`) |

## For Maintainers

| Workflow | Schedule | Description |
|----------|----------|-------------|
| Refresh Metadata | Weekly (Mon) | Updates stars, activity, and renamed repositories, removes repositories gone for a week, regenerates the README, and opens a PR |
| Check Links | Weekly (Mon) | Opens a PR fixing redirects and dead links, or an issue for links it cannot fix |
| Stale Author-Action Issues | Daily | Marks `question` suggestions stale after 14 days and closes them 14 days later |
| Triage Suggestions | On suggestion events; monthly (1st) | Applies the acceptance bar, opens entry PRs, and re-checks `monitoring` issues |

Maintainer actions on suggestions:

- **Polish before merging**: entry PRs flag a name that differs from the repository and a
  description that was cut (it ends with "…"); fix those in the PR.
- **Category**: if the bot can't place a passing suggestion, set the issue's **Category**
  field; the bot then opens the PR.
- **Accept outside the gates**: add `maintainer-pick` to the issue.
- **Reverse a decline** (an entry PR closed without merging): reopen the issue and add
  `maintainer-pick`, or reopen the entry PR and then the issue.

Bot PRs opened with the default `GITHUB_TOKEN` run no pull-request CI at all (the triage
bot validates entries before committing). To have them run CI, add a repository secret
`CREATE_PR_TOKEN` with a fine-grained or GitHub App token that has `contents: write` and
`pull-requests: write`.

## Note for AI Agents

If you are an automated agent acting for someone, add 🤖🤖🤖 to the end of the issue title.
Suggestions still need a person to confirm them, and the automation will ask that person to
do so. Issues opened by bot accounts (GitHub Apps) are not processed; submit through the
person's account.
