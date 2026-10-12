"""Tests for removing deleted repositories in scripts/fetch_metadata.py."""

import sys
import tempfile
import unittest
from datetime import date
from pathlib import Path
from unittest import mock

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

import fetch_metadata  # noqa: E402


class GoneRepoTest(unittest.TestCase):
    def refresh(self, entries, today, results):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "x.yaml"
            path.write_text(yaml.dump(entries), encoding="utf-8")
            with mock.patch.object(fetch_metadata, "fetch_repo_metadata", side_effect=lambda r: results[r]), \
                    mock.patch.object(fetch_metadata.time, "sleep"):
                fetch_metadata.process_yaml_file(path, today=today)
            return yaml.safe_load(path.read_text(encoding="utf-8"))

    def test_marks_then_removes_after_a_week(self):
        entries = [{"name": "Gone", "github": "a/gone"}, {"name": "Alive", "github": "a/alive"}]
        results = {"a/gone": fetch_metadata.GONE, "a/alive": ({"stars": 5}, "a/alive")}
        first = self.refresh(entries, date(2026, 10, 12), results)
        self.assertEqual(first[0]["_meta"]["gone_since"], "2026-10-12")
        self.assertEqual(len(self.refresh(first, date(2026, 10, 15), results)), 2)  # grace period
        self.assertEqual([e["name"] for e in self.refresh(first, date(2026, 10, 19), results)], ["Alive"])

    def test_back_again_clears_the_mark(self):
        entries = [{"name": "Back", "github": "a/back", "_meta": {"gone_since": "2026-10-12"}}]
        back = self.refresh(entries, date(2026, 10, 19), {"a/back": ({"stars": 1}, "a/back")})
        self.assertEqual(back[0]["_meta"], {"stars": 1})


    def test_failed_commit_lookup_keeps_the_previous_date(self):
        entries = [{"name": "X", "github": "a/x", "_meta": {"id": 1, "stars": 1, "last_commit": "2026-01-02"}}]
        kept = self.refresh(entries, date(2026, 10, 19), {"a/x": ({"id": 1, "stars": 2}, "a/x")})
        self.assertEqual(kept[0]["_meta"], {"id": 1, "stars": 2, "last_commit": "2026-01-02"})
        # A link corrected to another repository doesn't inherit the old repository's date.
        moved = self.refresh(entries, date(2026, 10, 19), {"a/x": ({"id": 2, "stars": 2}, "a/x")})
        self.assertEqual(moved[0]["_meta"], {"id": 2, "stars": 2})


class LookupTest(unittest.TestCase):
    def test_lookup_follows_the_link_and_never_pushed_at(self):
        urls = []
        repo = {"id": 99, "full_name": "new/correct", "stargazers_count": 3, "default_branch": "main",
                "pushed_at": "2026-10-01T00:00:00Z"}
        with mock.patch.object(fetch_metadata, "fetch_json", side_effect=lambda url, *a: urls.append(url) or repo), \
                mock.patch.object(fetch_metadata, "fetch_default_branch_commit_date", return_value=None) as commits:
            meta, full_name = fetch_metadata.fetch_repo_metadata("new/correct")
        # A corrected link wins over a stale stored ID, and its repository's ID is recorded.
        self.assertEqual(urls, ["https://api.github.com/repos/new/correct"])
        self.assertEqual((meta["id"], full_name), (99, "new/correct"))
        self.assertEqual(commits.call_args[0][0], "new/correct")
        self.assertNotIn("last_commit", meta)  # never pushed_at, which counts other branches


if __name__ == "__main__":
    unittest.main()
