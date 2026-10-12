"""The validator must reject data the README generator would silently drop."""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

import validate_entries  # noqa: E402


class DroppedDataTest(unittest.TestCase):
    def check(self, files: dict[str, str]) -> bool:
        with tempfile.TemporaryDirectory() as tmp:
            for name, text in files.items():
                (Path(tmp) / name).write_text(text, encoding="utf-8")
            return validate_entries.validate_entries(tmp)

    def test_valid_category(self):
        self.assertTrue(self.check({"control.yaml": "- name: Tool\n  github: o/r\n"}))

    def test_entries_without_a_list(self):
        self.assertFalse(self.check({"control.yaml": "name: Tool\ngithub: o/r\n"}))  # missing "- "

    def test_file_that_is_not_a_category(self):
        self.assertFalse(self.check({"physic-engines.yaml": "- name: Tool\n  github: o/r\n"}))


if __name__ == "__main__":
    unittest.main()
