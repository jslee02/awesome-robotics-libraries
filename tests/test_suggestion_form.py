"""The suggestion form's categories must match the generator: triage maps them by title."""

import sys
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))

from generate_readme import SECTIONS  # noqa: E402


class SuggestionFormTest(unittest.TestCase):
    def test_form_categories_match_generator(self):
        form = yaml.safe_load((ROOT / ".github" / "ISSUE_TEMPLATE" / "suggest-resource.yml").read_text(encoding="utf-8"))
        options = next(b for b in form["body"] if b.get("id") == "category")["attributes"]["options"]
        self.assertEqual(options, [title for _, title, _ in SECTIONS] + ["Not sure"])


if __name__ == "__main__":
    unittest.main()
