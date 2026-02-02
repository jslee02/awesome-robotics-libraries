#!/usr/bin/env python3
"""
Validate all data/*.yaml files against schema/entry.schema.json
"""

import json
import sys
from pathlib import Path

import jsonschema
import yaml


def load_schema(schema_path):
    """Load JSON schema from file."""
    with open(schema_path, "r") as f:
        return json.load(f)


def load_yaml_entries(yaml_path):
    """Load YAML file and return list of entries."""
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
        return data if isinstance(data, list) else []


def validate_entries():
    """Validate all data/*.yaml files against schema."""
    repo_root = Path(__file__).parent.parent
    schema_path = repo_root / "schema" / "entry.schema.json"
    data_dir = repo_root / "data"

    # Load schema
    try:
        schema = load_schema(schema_path)
    except Exception as e:
        print(f"ERROR: Failed to load schema: {e}", file=sys.stderr)
        return False

    # Find all YAML files
    yaml_files = sorted(data_dir.glob("*.yaml"))
    if not yaml_files:
        print(f"WARNING: No YAML files found in {data_dir}")
        return True

    all_valid = True
    total_entries = 0
    total_errors = 0

    for yaml_file in yaml_files:
        try:
            entries = load_yaml_entries(yaml_file)
        except Exception as e:
            print(f"ERROR: Failed to load {yaml_file.name}: {e}", file=sys.stderr)
            all_valid = False
            continue

        for idx, entry in enumerate(entries):
            total_entries += 1

            # Skip if entry is not a dict
            if not isinstance(entry, dict):
                print(
                    f"ERROR: {yaml_file.name}[{idx}]: Entry is not a dictionary",
                    file=sys.stderr,
                )
                all_valid = False
                total_errors += 1
                continue

            # Get entry name for error reporting
            entry_name = entry.get("name", f"[entry {idx}]")

            # Validate against schema (FormatChecker enforces format
            # constraints like "uri" and "date" declared in the schema)
            try:
                jsonschema.validate(
                    instance=entry,
                    schema=schema,
                    format_checker=jsonschema.FormatChecker(),
                )
            except jsonschema.ValidationError as e:
                print(
                    f"ERROR: {yaml_file.name}[{entry_name}]: {e.message}",
                    file=sys.stderr,
                )
                all_valid = False
                total_errors += 1
            except jsonschema.SchemaError as e:
                print(
                    f"ERROR: Schema error in {yaml_file.name}: {e.message}",
                    file=sys.stderr,
                )
                all_valid = False
                total_errors += 1

    # Print summary
    if all_valid:
        print(f"✓ All {total_entries} entries validated successfully")
        return True
    else:
        print(
            f"✗ Validation failed: {total_errors} error(s) in {total_entries} entries",
            file=sys.stderr,
        )
        return False


if __name__ == "__main__":
    success = validate_entries()
    sys.exit(0 if success else 1)
