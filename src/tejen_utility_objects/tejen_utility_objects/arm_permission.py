"""Small ROS-independent arm-interlock adapter used by CallbackManager.

Existing controllers that do not expose ``can_arm`` retain the historical behavior.
A controller may opt in by implementing ``can_arm()`` and returning either a bool or
``(allowed, reason)``.
"""

from __future__ import annotations


def arm_permission_for_node(node) -> tuple[bool, str]:
    checker = getattr(node, "can_arm", None)
    if checker is None:
        return True, ""
    result = checker()
    if isinstance(result, tuple):
        if len(result) != 2:
            raise ValueError("can_arm() tuple must contain exactly (allowed, reason)")
        return bool(result[0]), str(result[1])
    return bool(result), ""
