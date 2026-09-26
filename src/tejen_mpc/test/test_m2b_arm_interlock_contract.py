"""Contract for the optional generic arm-permission hook used by M2B."""

from tejen_utility_objects.arm_permission import arm_permission_for_node


class PlainNode:
    pass


class AllowedNode:
    def can_arm(self):
        return True, "bootstrap detached"


class BlockedNode:
    def can_arm(self):
        return False, "M2B bootstrap detach not verified"


def test_existing_nodes_without_hook_keep_existing_arm_permission():
    allowed, reason = arm_permission_for_node(PlainNode())
    assert allowed is True
    assert reason == ""


def test_node_arm_hook_can_allow_or_block_with_reason():
    assert arm_permission_for_node(AllowedNode()) == (True, "bootstrap detached")
    assert arm_permission_for_node(BlockedNode()) == (
        False,
        "M2B bootstrap detach not verified",
    )
