from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src" / "octopus_search.cpp"


def source_text() -> str:
    return SOURCE.read_text(encoding="utf-8")


def test_closed_set_identity_tracks_stage_and_three_cubic_history_voxels():
    source = source_text()

    state_key = source.split("struct StateKey {", 1)[1].split("struct StateKeyHash", 1)[0]
    assert "int index = 0;" in state_key
    assert "VoxelKey qi_m2;" in state_key
    assert "VoxelKey qi_m1;" in state_key
    assert "VoxelKey qi;" in state_key
    assert "index == other.index" in state_key
    assert "qi_m2 == other.qi_m2" in state_key
    assert "qi_m1 == other.qi_m1" in state_key
    assert "qi == other.qi" in state_key

    assert "std::unordered_set<StateKey, StateKeyHash> visited;" in source
    assert "std::unordered_set<VoxelKey, VoxelKeyHash> visited;" not in source
    assert "const StateKey key = stateKey(current);" in source


def test_state_history_matches_the_cubic_future_markov_history():
    source = source_text()
    history = source.split("const auto stateHistory", 1)[1].split(
        "const auto stateKey", 1
    )[0]

    # At q2 the history is the fixed splice prefix q0,q1,q2.
    assert "if (node->index == 2)" in history
    assert "history[0] = q012_.row(0).transpose();" in history
    assert "history[1] = q012_.row(1).transpose();" in history

    # At q3 the relevant predecessor pair is fixed q1 plus parent q2.
    assert "else if (node->index == 3)" in history
    assert "history[0] = q012_.row(1).transpose();" in history
    assert "history[1] = node->previous->qi;" in history

    # Later stages use the two actual predecessor nodes.
    assert "history[0] = node->previous->previous->qi;" in history
    assert "history[1] = node->previous->qi;" in history
    assert "history[2] = node->qi;" in history

    # Dynamics and pruning must consume the same history representation.
    assert "const auto history = stateHistory(current);" in source
    assert "current->index, history[0], history[1], history[2]" in source


def test_step1_does_not_replace_certification_or_fallback_selection():
    source = source_text()

    # The controlled experiment changes state equivalence only. Keep the existing
    # closest-complete/closest-partial selection and full final certification path.
    assert "closest_complete != nullptr ? closest_complete" in source
    assert ': closest;' in source
    assert "const ControlPoints completed = completedControlPoints(chosen);" in source
    assert "auto [final_feasible, separators] = finalCheck(completed);" in source
    assert "finalDynamicsFeasible(completed)" in source
