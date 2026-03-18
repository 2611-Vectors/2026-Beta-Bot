#!/usr/bin/env python3
"""
flip_path.py — Flip PathPlanner paths from top to bottom (mirror Y axis).

Usage:
    python flip_path.py <input.path> [output.path]

If no output file is given, the result is saved as <input>_flipped.path
"""

import json
import sys
import copy
from pathlib import Path

# Standard FRC field height in meters (2023–2025 fields are 8.21 m tall)
FIELD_HEIGHT_METERS = 8.069


def flip_y(y: float) -> float:
    """Mirror a Y coordinate across the horizontal center of the field."""
    return FIELD_HEIGHT_METERS - y


def flip_point(point: dict | None) -> dict | None:
    """Flip a {x, y} dict in-place (returns None if input is None)."""
    if point is None:
        return None
    return {"x": point["x"], "y": flip_y(point["y"])}


def flip_rotation(degrees: float) -> float:
    """
    Mirror a heading angle when flipping top-to-bottom.
    Angles pointing 'up' (positive Y) become 'down' (negative Y), etc.
    For a Y-flip: negate the angle, but keep it in (-180, 180].
    """
    flipped = -degrees
    # Normalise to (-180, 180]
    while flipped > 180:
        flipped -= 360
    while flipped <= -180:
        flipped += 360
    return flipped


def flip_path(data: dict) -> dict:
    """Return a deep-copied, Y-flipped version of a PathPlanner path dict."""
    flipped = copy.deepcopy(data)

    # ── Waypoints ──────────────────────────────────────────────────────────
    for wp in flipped.get("waypoints", []):
        wp["anchor"] = flip_point(wp["anchor"])
        if wp.get("prevControl") is not None:
            wp["prevControl"] = flip_point(wp["prevControl"])
        if wp.get("nextControl") is not None:
            wp["nextControl"] = flip_point(wp["nextControl"])

    # ── Rotation targets ───────────────────────────────────────────────────
    for rt in flipped.get("rotationTargets", []):
        if "rotationDegrees" in rt:
            rt["rotationDegrees"] = flip_rotation(rt["rotationDegrees"])

    # ── Goal end state rotation ────────────────────────────────────────────
    goal = flipped.get("goalEndState", {})
    if "rotation" in goal:
        goal["rotation"] = flip_rotation(goal["rotation"])

    # ── Ideal starting state rotation ─────────────────────────────────────
    ideal = flipped.get("idealStartingState", {})
    if "rotation" in ideal:
        ideal["rotation"] = flip_rotation(ideal["rotation"])

    # ── Preview start pose (shown in the GUI) ─────────────────────────────
    preview = flipped.get("previewStartingState", {})
    if "rotation" in preview:
        preview["rotation"] = flip_rotation(preview["rotation"])

    # ── Event markers (they store a waypoint relative position, no coords) ─
    # Event markers use `waypointRelativePos`, which is a 0–N float that
    # refers to a position along the path — no Y coordinate to flip.

    return flipped


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    input_path = Path(sys.argv[1])
    if not input_path.exists():
        print(f"Error: file not found: {input_path}")
        sys.exit(1)

    if len(sys.argv) >= 3:
        output_path = Path(sys.argv[2])
    else:
        output_path = input_path.with_name(input_path.stem + "_flipped" + input_path.suffix)

    with open(input_path, "r") as f:
        data = json.load(f)

    flipped = flip_path(data)

    with open(output_path, "w") as f:
        json.dump(flipped, f, indent=2)

    print(f"✓  Flipped path saved to: {output_path}")


if __name__ == "__main__":
    main()
