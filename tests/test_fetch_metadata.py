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


if __name__ == "__main__":
    unittest.main()
