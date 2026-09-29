#!/usr/bin/env python3
"""Revert only the two M2A scaffold changes that replaced/altered the X3 test model.

This helper is intentionally surgical:
  1. remove the exact kinematic marker block added by the earlier M2A hotfix;
  2. delete the placeholder m2a_kinematic_carrier.sdf introduced by the bench attempt.

It will not remove an unrecognized X3/base_link kinematic setting.
"""

from pathlib import Path
import re


REPO = Path(__file__).resolve().parents[2]
M2_MODEL = REPO / "simulation_assets" / "tejen" / "modelLargeM2BallMagnet.sdf"
PLACEHOLDER = REPO / "simulation_assets" / "m2a_kinematic_carrier.sdf"

MARKER_BLOCK = (
    "            <!-- M2A test scaffold: hold the quad body kinematically while the tether/magnet remain dynamic. -->\n"
    "            <kinematic>true</kinematic>\n"
)


def main():
    if not M2_MODEL.exists():
        raise SystemExit(f"missing expected M2 model: {M2_MODEL}")

    text = M2_MODEL.read_text(encoding="utf-8")
    if MARKER_BLOCK in text:
        text = text.replace(MARKER_BLOCK, "", 1)
        M2_MODEL.write_text(text, encoding="utf-8")
        print("Reverted prior M2A X3/base_link kinematic scaffold.")
    else:
        match = re.search(r'<link name="X3/base_link">(.*?)</link>', text, flags=re.S)
        if match and "<kinematic>true</kinematic>" in match.group(1):
            raise SystemExit(
                "X3/base_link is kinematic but the exact prior M2A marker is absent; "
                "refusing to modify an unknown model edit."
            )
        print("Prior M2A X3/base_link kinematic scaffold already absent.")

    if PLACEHOLDER.exists():
        PLACEHOLDER.unlink()
        print("Removed placeholder simulation_assets/m2a_kinematic_carrier.sdf.")
    else:
        print("Placeholder M2A carrier already absent.")


if __name__ == "__main__":
    main()
