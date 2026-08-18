#!/usr/bin/env python3
"""Update WPILib vendordep JSON files from the WPILib vendordep marketplace.

This mirrors what the WPILib VS Code extension's dependency manager does:
fetch the marketplace manifest for the frcYear, match each local vendordep
by uuid, and if the marketplace lists a newer version, download that exact
file.

Vendordeps whose uuid is not in the marketplace (e.g. WPILibNewCommands,
which ships with WPILib itself) are reported and left alone.

Writes a markdown summary of what changed to the path given by
--summary (used as the pull request body), and prints it to stdout.
"""

import argparse
import json
import re
import sys
import urllib.request
from pathlib import Path

MARKETPLACE_ROOT = "https://frcmaven.wpi.edu/artifactory/vendordeps/vendordep-marketplace"
VENDORDEPS_DIR = Path(__file__).resolve().parents[2] / "vendordeps"
TIMEOUT_SECONDS = 30


def fetch(url: str) -> str:
    req = urllib.request.Request(url)
    with urllib.request.urlopen(req, timeout=TIMEOUT_SECONDS) as resp:
        return resp.read().decode("utf-8")


def version_key(version: str) -> tuple:
    # Converts version string into a tuple for easier comparison
    return tuple(int(part) for part in re.findall(r"\d+", version))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary", type=Path, help="Path to write the markdown summary to")
    args = parser.parse_args()

    manifests: dict[str, list] = {}
    updated: list[str] = []
    unchanged: list[str] = []
    skipped: list[str] = []
    errors: list[str] = []

    for path in sorted(VENDORDEPS_DIR.glob("*.json")):
        local = json.loads(path.read_text())
        name = local.get("name", path.name)
        uuid = local.get("uuid")
        year = local.get("frcYear")
        local_version = local.get("version", "")

        if not uuid or not year:
            skipped.append(f"**{name}** (`{path.name}`): missing uuid or frcYear")
            continue

        if year not in manifests:
            try:
                manifests[year] = json.loads(fetch(f"{MARKETPLACE_ROOT}/{year}.json"))
            except Exception as e:  # noqa: BLE001 (network/remote errors; handled and reported below)
                manifests[year] = None
                errors.append(f"Failed to fetch the {year} marketplace manifest ({e})")

        if manifests.get(year) is None:
            skipped.append(
                f"**{name}** (`{path.name}`): skipped because the {year} marketplace manifest could not be fetched"
            )
            continue

        candidates = [e for e in manifests[year] if e.get("uuid") == uuid]  # WPILib repo keeps multiple versions
        if not candidates:
            skipped.append(f"**{name}** (`{path.name}`): not in the {year} marketplace")
            continue

        best = max(candidates, key=lambda e: version_key(e.get("version", "")))
        best_version = best.get("version", "")

        if version_key(best_version) <= version_key(local_version):
            unchanged.append(f"**{name}**: {local_version}")
            continue

        try:
            new_text = fetch(f"{MARKETPLACE_ROOT}/{best['path']}")
            new_json = json.loads(new_text)
        except Exception as e:  # noqa: BLE001
            errors.append(f"**{name}**: failed to download {best['path']} ({e})")
            continue

        new_path = VENDORDEPS_DIR / new_json.get("fileName", Path(best["path"]).name)
        try:
            new_path.write_text(new_text)
            if new_path != path:
                path.unlink() # delete the old file
        except OSError as e:
            errors.append(f"**{name}**: failed to write `{new_path.name}` ({e})")
            continue

        updated.append(f"**{name}**: {local_version} -> {best_version} (`{new_path.name}`)")

    lines = []
    if updated:
        lines.append("## Updated")
        lines += [f"- {s}" for s in updated]
    if errors:
        lines.append("\n## Errors (left unchanged)")
        lines += [f"- {s}" for s in errors]
    if unchanged:
        lines.append("\n## Already up to date")
        lines += [f"- {s}" for s in unchanged]
    if skipped:
        lines.append("\n## Skipped")
        lines += [f"- {s}" for s in skipped]
    summary = "\n".join(lines) + "\n"

    print(summary)
    if args.summary:
        args.summary.write_text(summary)

    return 0


if __name__ == "__main__":
    sys.exit(main())
