"""Shared ACADOS artifact cache helpers for the payload MPC.

The runtime solver objects remain process-local.  Only generated C / JSON / shared
libraries are cached so identical controller processes do not compile the same
solver repeatedly.
"""

from __future__ import annotations

from contextlib import contextmanager
import fcntl
import hashlib
import json
from pathlib import Path
from typing import Iterable, NamedTuple


CACHE_SCHEMA_VERSION = 1


class PayloadMpcCacheIdentity(NamedTuple):
    fingerprint: str
    manifest_payload: dict


class PayloadMpcCacheLayout(NamedTuple):
    root: Path
    code_export_dir: Path
    ocp_json: Path
    sim_json: Path
    ocp_shared_library: Path
    sim_shared_library: Path
    manifest: Path
    lock: Path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_payload_mpc_cache_identity(
    *,
    source_paths: Iterable[Path],
    xy_integral_weight: float,
    xy_integral_terminal_weight: float,
) -> PayloadMpcCacheIdentity:
    """Fingerprint every input that changes generated payload-MPC artifacts."""
    sources = {}
    for raw_path in source_paths:
        path = Path(raw_path).resolve()
        sources[path.name] = {
            "sha256": _sha256(path),
            "size_bytes": path.stat().st_size,
        }

    payload = {
        "schema_version": CACHE_SCHEMA_VERSION,
        "solver_family": "quad_payload_integral_dynamics",
        "xy_integral_weight": float(xy_integral_weight),
        "xy_integral_terminal_weight": float(xy_integral_terminal_weight),
        "sources": dict(sorted(sources.items())),
    }
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    fingerprint = hashlib.sha256(canonical).hexdigest()
    manifest = dict(payload)
    manifest["fingerprint"] = fingerprint
    return PayloadMpcCacheIdentity(fingerprint=fingerprint, manifest_payload=manifest)


def payload_mpc_cache_layout(base_dir: Path, fingerprint: str) -> PayloadMpcCacheLayout:
    """Return the isolated cache entry for one exact generated-solver identity."""
    root = Path(base_dir).expanduser().resolve() / fingerprint[:20]
    code_export_dir = root / "c_generated_code"
    return PayloadMpcCacheLayout(
        root=root,
        code_export_dir=code_export_dir,
        ocp_json=root / "quad_payload_integral_dynamics_ocp.json",
        sim_json=root / "quad_payload_dynamics_sim_sim.json",
        ocp_shared_library=(
            code_export_dir / "libacados_ocp_solver_quad_payload_integral_dynamics.so"
        ),
        sim_shared_library=(
            code_export_dir / "libacados_sim_solver_quad_payload_dynamics_sim.so"
        ),
        manifest=root / "cache_manifest.json",
        lock=root / ".compile.lock",
    )


def payload_mpc_cache_is_reusable(
    layout: PayloadMpcCacheLayout,
    identity: PayloadMpcCacheIdentity,
) -> bool:
    """Require an exact manifest match plus both JSON/shared-library pairs."""
    try:
        manifest = json.loads(layout.manifest.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    if manifest != identity.manifest_payload:
        return False
    required = (
        layout.ocp_json,
        layout.sim_json,
        layout.ocp_shared_library,
        layout.sim_shared_library,
    )
    return all(path.is_file() and path.stat().st_size > 0 for path in required)


def write_payload_mpc_cache_manifest(
    layout: PayloadMpcCacheLayout,
    identity: PayloadMpcCacheIdentity,
) -> None:
    """Atomically mark a fully generated cache entry reusable."""
    layout.root.mkdir(parents=True, exist_ok=True)
    tmp = layout.manifest.with_suffix(".json.tmp")
    tmp.write_text(
        json.dumps(identity.manifest_payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    tmp.replace(layout.manifest)


@contextmanager
def locked_payload_mpc_cache(base_dir: Path, fingerprint: str):
    """Serialize generation for one cache identity, matching Wesley's lock pattern."""
    layout = payload_mpc_cache_layout(base_dir, fingerprint)
    layout.root.mkdir(parents=True, exist_ok=True)
    lock_handle = layout.lock.open("a+")
    try:
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX)
        yield layout
    finally:
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
        lock_handle.close()
