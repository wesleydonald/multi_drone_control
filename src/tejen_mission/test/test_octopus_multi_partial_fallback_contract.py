from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
CPP = ROOT / "src/tejen_dynamic_planner/standalone/src/octopus_search.cpp"
HPP = ROOT / "src/tejen_dynamic_planner/standalone/include/dynamic_planner/octopus_search.hpp"
BACKEND = ROOT / "src/tejen_dynamic_planner/src/transfer_backend_node.cpp"


def test_multi_partial_fallback_is_bounded_and_preserves_complete_precedence():
    source = CPP.read_text()
    assert "kPartialFallbackCandidateLimit = 8U" in source
    assert "partial_fallback_candidates.resize(kPartialFallbackCandidateLimit)" in source
    assert "considerPartialFallback(start, start->h);" in source
    assert "already_retained = std::any_of" in source
    assert "Node* chosen_complete = goal_node != nullptr ? goal_node : closest_complete;" in source
    assert "if (chosen_complete != nullptr)" in source
    assert "for (std::size_t rank = 0; rank < partial_fallback_candidates.size(); ++rank)" in source
    assert "completedControlPoints(candidate)" in source
    assert "finalCheck(completed)" in source
    assert "finalDynamicsFeasible(completed)" in source
    assert "terminalHoldViable(completed.row(completed.rows() - 1).transpose())" in source


def test_partial_fallback_keeps_existing_external_status_and_exposes_rank_diagnostics():
    source = CPP.read_text()
    header = HPP.read_text()
    backend = BACKEND.read_text()

    assert 'result.status = "PADDED_CLOSEST_PARTIAL";' in source
    assert 'result.status = "PADDED_CLOSEST_PARTIAL_FINAL_CHECK_FAILED";' in source
    assert "partial_fallback_candidates_retained" in header
    assert "partial_fallback_candidates_tested" in header
    assert "partial_fallback_selected_rank" in header
    assert "partial_fallback_retained,partial_fallback_tested" in backend
    assert "partial_fallback_selected_rank" in backend
