#!/usr/bin/env python3
"""Helpers for fetching GitHub repository metadata."""

from __future__ import annotations

import json
import urllib.error
import urllib.parse
import urllib.request

API_BASE = "https://api.github.com"


def github_headers(token: str | None = None, user_agent: str = "") -> dict[str, str]:
    headers = {
        "Accept": "application/vnd.github+json",
        "X-GitHub-Api-Version": "2022-11-28",
        "User-Agent": user_agent or "awesome-robotics-libraries-bot",
    }
    if token:
        headers["Authorization"] = f"Bearer {token}"
    return headers


def fetch_json(
    url: str,
    token: str | None = None,
    user_agent: str = "",
) -> dict | list | None:
    req = urllib.request.Request(url, headers=github_headers(token, user_agent))
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            return json.loads(resp.read())
    except urllib.error.HTTPError:
        raise
    except (urllib.error.URLError, TimeoutError):
        raise


def fetch_default_branch_commit_date(
    owner_repo: str,
    default_branch: str,
    token: str | None = None,
    user_agent: str = "",
) -> str | None:
    branch_ref = urllib.parse.quote(default_branch, safe="")
    data = fetch_json(
        f"{API_BASE}/repos/{owner_repo}/commits?per_page=1&sha={branch_ref}",
        token,
        user_agent,
    )
    if not isinstance(data, list) or not data:
        return None

    commit = data[0].get("commit", {})
    committer = commit.get("committer") or {}
    author = commit.get("author") or {}
    commit_date = (committer.get("date") or author.get("date") or "")[:10]
    return commit_date or None
