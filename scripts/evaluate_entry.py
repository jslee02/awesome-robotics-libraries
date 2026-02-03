#!/usr/bin/env python3
"""Evaluate a GitHub repository against the inclusion criteria.

Usage:
    python3 scripts/evaluate_entry.py owner/repo
    python3 scripts/evaluate_entry.py --json owner/repo

Exit codes:
    0 = accept, 1 = incubator, 2 = reject, 3 = error
"""

from __future__ import annotations

import argparse
import json
import sys
import urllib.error
import urllib.request
from datetime import date, timedelta

API_BASE = "https://api.github.com"


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


def evaluate(owner_repo: str, token: str | None = None) -> dict:
    """Evaluate a GitHub repo and return structured results."""
    data = _fetch_json(f"{API_BASE}/repos/{owner_repo}", token)
    if data is None:
        return {
            "repo": owner_repo,
            "error": "Could not fetch repository data",
            "recommendation": "error",
        }

    if data.get("message") == "Not Found":
        return {
            "repo": owner_repo,
            "error": "Repository not found",
            "recommendation": "error",
        }

    today = date.today()
    stars = data.get("stargazers_count", 0)
    archived = data.get("archived", False)
    created_at = data.get("created_at", "")[:10]
    pushed_at = data.get("pushed_at", "")[:10]
    has_license = bool(
        data.get("license")
        and data["license"].get("spdx_id", "NOASSERTION") != "NOASSERTION"
    )
    description = data.get("description") or ""
    has_homepage = bool(data.get("homepage"))

    # Fetch README existence
    readme_data = _fetch_json(f"{API_BASE}/repos/{owner_repo}/readme", token)
    has_readme = readme_data is not None and "content" in (readme_data or {})

    # --- Scoring checklist ---
    checks: dict[str, dict] = {}

    # 1. Popularity: ≥ 50 stars
    checks["popularity"] = {
        "pass": stars >= 50,
        "detail": f"{stars} stars (threshold: 50)",
    }

    # 2. Activity: commit within last 2 years
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

    # 3. Documentation: has README
    checks["documentation"] = {
        "pass": has_readme,
        "detail": "README found" if has_readme else "no README",
    }

    # 4. Maturity: ≥ 6 months old
    mature = False
    if created_at:
        try:
            creation = date.fromisoformat(created_at)
            mature = (today - creation).days >= 180
        except ValueError:
            pass
    checks["maturity"] = {
        "pass": mature,
        "detail": f"created {created_at}"
        + (
            f" ({(today - date.fromisoformat(created_at)).days} days ago)"
            if created_at
            else ""
        ),
    }

    # 5. Uniqueness: cannot be auto-evaluated
    checks["uniqueness"] = {
        "pass": None,  # manual review required
        "detail": "requires manual review",
    }

    # Count passing checks (None = unknown, doesn't count)
    auto_score = sum(1 for c in checks.values() if c["pass"] is True)
    auto_total = sum(1 for c in checks.values() if c["pass"] is not None)

    # Recommendation
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

    return {
        "repo": owner_repo,
        "name": data.get("full_name", owner_repo),
        "description": description,
        "stars": stars,
        "archived": archived,
        "created": created_at,
        "last_push": pushed_at,
        "has_readme": has_readme,
        "has_license": has_license,
        "has_homepage": has_homepage,
        "checks": checks,
        "auto_score": auto_score,
        "auto_total": auto_total,
        "recommendation": recommendation,
        "reason": reason,
    }


def format_report(result: dict) -> str:
    """Format evaluation result as a human-readable Markdown report."""
    if "error" in result:
        return f"❌ **Error**: {result['error']}\n"

    lines = []
    lines.append(f"## Evaluation: {result['name']}")
    lines.append("")

    if result["description"]:
        lines.append(f"> {result['description']}")
        lines.append("")

    # Metadata summary
    lines.append("### Repository Info")
    lines.append("")
    lines.append(f"| Field | Value |")
    lines.append(f"|-------|-------|")
    lines.append(f"| Stars | {result['stars']:,} |")
    lines.append(f"| Created | {result['created']} |")
    lines.append(f"| Last Push | {result['last_push']} |")
    lines.append(f"| Archived | {'⚠️ Yes' if result['archived'] else 'No'} |")
    lines.append(f"| README | {'✅' if result['has_readme'] else '❌'} |")
    lines.append(f"| License | {'✅' if result['has_license'] else '❌'} |")
    lines.append(f"| Homepage | {'✅' if result['has_homepage'] else '—'} |")
    lines.append("")

    # Scoring
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

    # Recommendation
    rec = result["recommendation"]
    if rec == "accept":
        emoji = "✅"
        label = "Accept"
    elif rec == "likely_accept":
        emoji = "🟢"
        label = "Likely Accept"
    elif rec == "incubator":
        emoji = "🟡"
        label = "Incubator"
    else:
        emoji = "❌"
        label = "Reject"

    lines.append(f"### Recommendation: {emoji} {label}")
    lines.append("")
    lines.append(result["reason"])
    lines.append("")
    lines.append("---")
    lines.append(
        "*Auto-evaluated by [evaluate_entry.py](../scripts/evaluate_entry.py). Uniqueness and relevance require manual review.*"
    )

    return "\n".join(lines)


def main():
    import os

    parser = argparse.ArgumentParser(
        description="Evaluate a GitHub repo for inclusion in the awesome list"
    )
    parser.add_argument("repo", help="GitHub owner/repo (e.g., google-deepmind/mujoco)")
    parser.add_argument(
        "--json", action="store_true", dest="json_output", help="Output raw JSON"
    )
    parser.add_argument("--token", default=None, help="GitHub API token")
    args = parser.parse_args()

    token = args.token or os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")
    result = evaluate(args.repo, token)

    if args.json_output:
        print(json.dumps(result, indent=2))
    else:
        print(format_report(result))

    # Exit code based on recommendation
    rec = result.get("recommendation", "error")
    if rec in ("accept", "likely_accept"):
        sys.exit(0)
    elif rec == "incubator":
        sys.exit(1)
    elif rec == "error":
        sys.exit(3)
    else:
        sys.exit(2)


if __name__ == "__main__":
    main()
