#!/usr/bin/env python3
"""Fetch GitHub metadata for all entries in data/*.yaml and update _meta fields."""

from __future__ import annotations

import json
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import yaml

API_BASE = "https://api.github.com"
RATE_LIMIT_PAUSE = 2  # seconds between requests to avoid abuse detection


def _github_headers() -> dict[str, str]:
    headers = {
        "Accept": "application/vnd.github+json",
        "X-GitHub-Api-Version": "2022-11-28",
        "User-Agent": "awesome-robotics-libraries-metadata-bot",
    }
    token = os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")
    if token:
        headers["Authorization"] = f"Bearer {token}"
    return headers


def fetch_repo_metadata(owner_repo: str) -> dict | None:
    url = f"{API_BASE}/repos/{owner_repo}"
    req = urllib.request.Request(url, headers=_github_headers())
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            data = json.loads(resp.read())
    except urllib.error.HTTPError as e:
        if e.code == 404:
            print(f"  WARN: {owner_repo} — 404 Not Found", file=sys.stderr)
        elif e.code == 403:
            print(f"  WARN: {owner_repo} — 403 rate limited", file=sys.stderr)
        else:
            print(f"  WARN: {owner_repo} — HTTP {e.code}", file=sys.stderr)
        return None
    except (urllib.error.URLError, TimeoutError) as e:
        print(f"  WARN: {owner_repo} — {e}", file=sys.stderr)
        return None

    meta: dict = {}
    if "stargazers_count" in data:
        meta["stars"] = data["stargazers_count"]
    if data.get("pushed_at"):
        meta["last_commit"] = data["pushed_at"][:10]
    if data.get("archived") is True:
        meta["archived"] = True
    if data.get("license") and data["license"].get("spdx_id"):
        spdx = data["license"]["spdx_id"]
        if spdx != "NOASSERTION":
            meta["license"] = spdx
    if data.get("language"):
        meta["language"] = data["language"]
    return meta


def process_yaml_file(yaml_path: Path, dry_run: bool = False) -> tuple[int, int]:
    with open(yaml_path, encoding="utf-8") as f:
        entries = yaml.safe_load(f)
    if not isinstance(entries, list):
        return 0, 0

    updated = 0
    total = 0
    for entry in entries:
        github = entry.get("github")
        if not github:
            continue
        total += 1
        print(f"  {github}...", end=" ", flush=True)

        meta = fetch_repo_metadata(github)
        if meta:
            entry["_meta"] = meta
            updated += 1
            stars = meta.get("stars", "?")
            print(f"★ {stars}")
        else:
            print("skipped")

        time.sleep(RATE_LIMIT_PAUSE)

    if not dry_run and updated > 0:
        with open(yaml_path, "w", encoding="utf-8") as f:
            yaml.dump(
                entries,
                f,
                default_flow_style=False,
                allow_unicode=True,
                sort_keys=False,
                width=120,
            )

    return total, updated


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Fetch GitHub metadata for YAML entries"
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="Print metadata without writing"
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    data_dir = repo_root / "data"

    if not data_dir.exists():
        print(f"ERROR: {data_dir} not found", file=sys.stderr)
        sys.exit(1)

    token = os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")
    if not token:
        print("WARNING: No GITHUB_TOKEN set — rate limit is 60 req/hr", file=sys.stderr)

    yaml_files = sorted(data_dir.glob("*.yaml"))
    grand_total = 0
    grand_updated = 0

    for yaml_file in yaml_files:
        print(f"\n{yaml_file.name}:")
        total, updated = process_yaml_file(yaml_file, dry_run=args.dry_run)
        grand_total += total
        grand_updated += updated

    print(f"\nDone: {grand_updated}/{grand_total} entries updated")
    if args.dry_run:
        print("(dry run — no files written)")


if __name__ == "__main__":
    main()
