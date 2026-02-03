#!/usr/bin/env python3
"""Evaluate a GitHub repository against the inclusion criteria.

Usage:
    python3 scripts/evaluate_entry.py owner/repo
    python3 scripts/evaluate_entry.py --json owner/repo
    python3 scripts/evaluate_entry.py --data-dir data/ owner/repo   # with duplicate check

Exit codes:
    0 = accept, 1 = incubator, 2 = reject, 3 = error
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.error
import urllib.request
from datetime import date
from pathlib import Path

import yaml

API_BASE = "https://api.github.com"

CATEGORY_TO_YAML = {
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
    "Simulators": "simulators",
    "Other Awesome Lists": "other-awesome-lists",
}

GUIDANCE = {
    "popularity": (
        "The project has fewer than 50 GitHub stars. "
        "This threshold helps ensure community adoption. "
        "As the project gains traction, it will automatically qualify. "
        "Consider sharing it in relevant communities to grow visibility."
    ),
    "activity": (
        "No commits detected in the last 2 years. "
        "We look for signs of active maintenance. "
        "Even a small release, bug fix, or documentation update would satisfy this criterion. "
        "If development has moved to a different repository, please let us know."
    ),
    "documentation": (
        "We could not find a README file. "
        "A README with a brief description, installation instructions, "
        "and a usage example makes it much easier for users to evaluate the project."
    ),
    "maturity": (
        "The project is less than 6 months old. "
        "We wait for projects to stabilize before listing them. "
        "This issue will be automatically re-evaluated once the project matures — no action needed from you."
    ),
}


def _github_headers(token: str | None = None) -> dict[str, str]:
    headers = {
        "Accept": "application/vnd.github+json",
        "X-GitHub-Api-Version": "2022-11-28",
        "User-Agent": "awesome-robotics-libraries-evaluator",
    }
    if token:
        headers["Authorization"] = f"Bearer {token}"
    return headers


def _fetch_json(url: str, token: str | None = None) -> dict | None:
    req = urllib.request.Request(url, headers=_github_headers(token))
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            return json.loads(resp.read())
    except urllib.error.HTTPError as e:
        print(f"ERROR: HTTP {e.code} for {url}", file=sys.stderr)
        return None
    except (urllib.error.URLError, TimeoutError) as e:
        print(f"ERROR: {e} for {url}", file=sys.stderr)
        return None


def find_duplicates(owner_repo: str, data_dir: Path) -> list[dict]:
    """Check if owner_repo already exists in any data/*.yaml file."""
    slug = owner_repo.lower()
    matches = []
    for yaml_path in sorted(data_dir.glob("*.yaml")):
        with open(yaml_path, encoding="utf-8") as f:
            entries = yaml.safe_load(f)
        if not isinstance(entries, list):
            continue
        for entry in entries:
            if entry.get("github", "").lower() == slug:
                matches.append(
                    {
                        "name": entry.get("name", ""),
                        "file": yaml_path.name,
                        "section": yaml_path.stem,
                    }
                )
    return matches


def evaluate(
    owner_repo: str,
    token: str | None = None,
    data_dir: Path | None = None,
) -> dict:
    """Evaluate a GitHub repo and return structured results."""
    result_base = {"repo": owner_repo}

    # --- Duplicate check ---
    duplicates = []
    if data_dir and data_dir.exists():
        duplicates = find_duplicates(owner_repo, data_dir)

    if duplicates:
        entry = duplicates[0]
        return {
            **result_base,
            "duplicate": True,
            "duplicate_of": entry["name"],
            "duplicate_section": entry["section"],
            "recommendation": "duplicate",
            "reason": f'Already listed as "{entry["name"]}" in {entry["file"]}',
        }

    data = _fetch_json(f"{API_BASE}/repos/{owner_repo}", token)
    if data is None:
        return {
            **result_base,
            "error": "Could not fetch repository data",
            "recommendation": "error",
        }

    if data.get("message") == "Not Found":
        return {
            **result_base,
            "error": "Repository not found",
            "recommendation": "error",
        }

    today = date.today()
    stars = data.get("stargazers_count", 0)
    archived = data.get("archived", False)
    created_at = data.get("created_at", "")[:10]
    pushed_at = data.get("pushed_at", "")[:10]
    license_spdx = None
    if (
        data.get("license")
        and data["license"].get("spdx_id", "NOASSERTION") != "NOASSERTION"
    ):
        license_spdx = data["license"]["spdx_id"]
    description = data.get("description") or ""
    has_homepage = bool(data.get("homepage"))
    homepage_url = data.get("homepage") or ""
    language = data.get("language") or ""

    readme_data = _fetch_json(f"{API_BASE}/repos/{owner_repo}/readme", token)
    has_readme = readme_data is not None and "content" in (readme_data or {})

    # --- Scoring checklist ---
    checks: dict[str, dict] = {}

    checks["popularity"] = {
        "pass": stars >= 50,
        "detail": f"{stars} stars (threshold: 50)",
    }

    active = False
    if pushed_at:
        try:
            last_push = date.fromisoformat(pushed_at)
            active = (today - last_push).days <= 730
        except ValueError:
            pass
    checks["activity"] = {
        "pass": active and not archived,
        "detail": f"last push {pushed_at}" + (" ⚠️ ARCHIVED" if archived else ""),
    }

    checks["documentation"] = {
        "pass": has_readme,
        "detail": "README found" if has_readme else "no README",
    }

    mature = False
    age_days = 0
    if created_at:
        try:
            creation = date.fromisoformat(created_at)
            age_days = (today - creation).days
            mature = age_days >= 180
        except ValueError:
            pass
    checks["maturity"] = {
        "pass": mature,
        "detail": f"created {created_at}"
        + (f" ({age_days} days ago)" if created_at else ""),
    }

    # Cannot be auto-evaluated
    checks["uniqueness"] = {
        "pass": None,
        "detail": "requires manual review",
    }

    auto_score = sum(1 for c in checks.values() if c["pass"] is True)
    auto_total = sum(1 for c in checks.values() if c["pass"] is not None)
    failed_checks = [name for name, c in checks.items() if c["pass"] is False]

    if archived:
        recommendation = "reject"
        reason = "Repository is archived"
    elif auto_score >= 3:
        recommendation = "accept"
        reason = f"Passes {auto_score}/{auto_total} auto-checkable criteria (uniqueness needs manual review)"
    elif auto_score >= 2:
        recommendation = "likely_accept"
        reason = f"Passes {auto_score}/{auto_total} auto-checkable criteria; if uniqueness is confirmed, meets ≥ 3 threshold"
    elif auto_score >= 1:
        recommendation = "incubator"
        reason = (
            f"Passes {auto_score}/{auto_total} auto-checkable criteria; needs to mature"
        )
    else:
        recommendation = "reject"
        reason = f"Passes {auto_score}/{auto_total} auto-checkable criteria"

    # _meta block ready for YAML insertion
    meta = {"stars": stars, "last_commit": pushed_at}
    if archived:
        meta["archived"] = True
    if license_spdx:
        meta["license"] = license_spdx
    if language:
        meta["language"] = language

    return {
        **result_base,
        "name": data.get("name", owner_repo.split("/")[-1]),
        "full_name": data.get("full_name", owner_repo),
        "description": description,
        "stars": stars,
        "archived": archived,
        "created": created_at,
        "last_push": pushed_at,
        "has_readme": has_readme,
        "has_license": license_spdx is not None,
        "license_spdx": license_spdx,
        "has_homepage": has_homepage,
        "homepage_url": homepage_url,
        "language": language,
        "checks": checks,
        "auto_score": auto_score,
        "auto_total": auto_total,
        "failed_checks": failed_checks,
        "recommendation": recommendation,
        "reason": reason,
        "meta": meta,
    }


def format_report(result: dict) -> str:
    """Format evaluation result as a human-readable Markdown report."""
    if "error" in result:
        return (
            f"👋 Thanks for the suggestion!\n\n"
            f"❌ **Error**: {result['error']}\n\n"
            f"Please double-check the repository URL and resubmit. "
            f"If you believe this is a mistake, feel free to comment and a maintainer will take a look."
        )

    if result.get("duplicate"):
        return (
            f"👋 Thanks for suggesting this project!\n\n"
            f"It looks like **{result['duplicate_of']}** is already on our list "
            f"(in the _{result['duplicate_section'].replace('-', ' ').title()}_ section).\n\n"
            f"If you think the existing entry needs updating (description, URL, etc.), "
            f"feel free to open a separate issue or PR. We appreciate you looking out for the list! 🙏"
        )

    lines = []
    lines.append(f"## 👋 Evaluation: {result['full_name']}")
    lines.append("")
    lines.append("Thanks for suggesting this project! Here's our automated evaluation:")
    lines.append("")

    if result["description"]:
        lines.append(f"> {result['description']}")
        lines.append("")

    lines.append("### Repository Info")
    lines.append("")
    lines.append("| Field | Value |")
    lines.append("|-------|-------|")
    lines.append(f"| Stars | {result['stars']:,} |")
    lines.append(f"| Created | {result['created']} |")
    lines.append(f"| Last Push | {result['last_push']} |")
    lines.append(f"| Archived | {'⚠️ Yes' if result['archived'] else 'No'} |")
    lines.append(f"| README | {'✅' if result['has_readme'] else '❌'} |")
    lines.append(
        f"| License | {'✅ ' + (result.get('license_spdx') or '') if result['has_license'] else '❌'} |"
    )
    lines.append(f"| Homepage | {'✅' if result['has_homepage'] else '—'} |")
    lines.append("")

    lines.append("### Scoring Checklist")
    lines.append("")
    for name, check in result["checks"].items():
        if check["pass"] is True:
            icon = "✅"
        elif check["pass"] is False:
            icon = "❌"
        else:
            icon = "🔍"
        lines.append(
            f"- {icon} **{name.replace('_', ' ').title()}**: {check['detail']}"
        )
    lines.append("")
    lines.append(
        f"**Auto-score**: {result['auto_score']}/{result['auto_total']} (excludes manual checks)"
    )
    lines.append("")

    rec = result["recommendation"]
    if rec == "accept":
        lines.append("### Recommendation: ✅ Accept")
        lines.append("")
        lines.append(result["reason"])
        lines.append("")
        lines.append(
            "A maintainer will review the uniqueness/relevance criterion and, "
            "if approved, a PR to add this entry will be generated automatically. "
            "Sit tight! 🚀"
        )
    elif rec == "likely_accept":
        lines.append("### Recommendation: 🟢 Likely Accept")
        lines.append("")
        lines.append(result["reason"])
        lines.append("")
        lines.append(
            "This is very close to meeting all criteria. "
            "A maintainer will verify uniqueness — if confirmed, it's in! "
            "No action needed from you."
        )
    elif rec == "incubator":
        lines.append("### Recommendation: 🟡 Incubator")
        lines.append("")
        lines.append(result["reason"])
        lines.append("")
        lines.append(
            "Don't worry — this issue will stay open and be **automatically re-evaluated monthly**. "
            "When the project meets enough criteria, it will be promoted automatically.\n"
        )
        failed = result.get("failed_checks", [])
        if failed:
            lines.append("**Here's what would help it qualify:**\n")
            for check_name in failed:
                guidance = GUIDANCE.get(check_name, "")
                if guidance:
                    lines.append(
                        f"- **{check_name.replace('_', ' ').title()}**: {guidance}"
                    )
            lines.append("")
    elif rec == "reject" and result.get("archived"):
        lines.append("### Recommendation: ⚠️ Archived Repository")
        lines.append("")
        lines.append(
            "This repository has been archived by its maintainer, which means "
            "it's no longer accepting contributions or updates.\n\n"
            "**Exceptions**: If this project has historical significance to the robotics community "
            "(like ODE or Simbody), please comment below explaining its importance and "
            "a maintainer will consider it.\n\n"
            "If development has moved to a new repository, feel free to suggest that one instead!"
        )
    else:
        lines.append("### Recommendation: 🟡 Needs More Growth")
        lines.append("")
        lines.append(result["reason"])
        lines.append("")
        lines.append(
            "This project doesn't meet enough criteria yet, but that's okay! "
            "This issue will stay open in our **incubator queue** and be "
            "**re-evaluated automatically every month**.\n"
        )
        failed = result.get("failed_checks", [])
        if failed:
            lines.append("**Here's what would help it qualify:**\n")
            for check_name in failed:
                guidance = GUIDANCE.get(check_name, "")
                if guidance:
                    lines.append(
                        f"- **{check_name.replace('_', ' ').title()}**: {guidance}"
                    )
            lines.append("")
        lines.append(
            "We genuinely appreciate you bringing this project to our attention. "
            "Many great libraries start small — we'll keep watching! 👀"
        )

    lines.append("")
    lines.append("---")
    lines.append(
        "*🤖 Auto-evaluated by [evaluate_entry.py](../scripts/evaluate_entry.py). "
        "Uniqueness and relevance require manual review by a maintainer.*"
    )

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate a GitHub repo for inclusion in the awesome list"
    )
    parser.add_argument("repo", help="GitHub owner/repo (e.g., google-deepmind/mujoco)")
    parser.add_argument(
        "--json", action="store_true", dest="json_output", help="Output raw JSON"
    )
    parser.add_argument("--token", default=None, help="GitHub API token")
    parser.add_argument(
        "--data-dir",
        default=None,
        help="Path to data/ directory for duplicate checking",
    )
    args = parser.parse_args()

    token = args.token or os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")
    data_dir = Path(args.data_dir) if args.data_dir else None

    result = evaluate(args.repo, token, data_dir)

    if args.json_output:
        print(json.dumps(result, indent=2))
    else:
        print(format_report(result))

    rec = result.get("recommendation", "error")
    if rec in ("accept", "likely_accept"):
        sys.exit(0)
    elif rec in ("incubator", "duplicate"):
        sys.exit(1)
    elif rec == "error":
        sys.exit(3)
    else:
        sys.exit(2)


if __name__ == "__main__":
    main()
