"""Locate workspace assets (verification videos, model weights) portably.

Launch defaults and node parameters historically hardcoded the Docker
bind-mount path "/aiformula_machine/..." (see compose.yaml: ".:/aiformula_machine").
That path only resolves inside the container; running the same code on the
host, or from a workspace checked out somewhere else, left every video/model
default broken (FileNotFoundError, or a silently empty video list). These
helpers search a short list of plausible workspace roots instead of assuming
one fixed location.
"""

import os

_ENV_VAR = "AIFORMULA_WS"
_DOCKER_MOUNT = "/aiformula_machine"
# aiformula_machine was split off from an aiformula_ws-based container/workspace;
# some checkouts (and a sibling dev workspace on this machine) still use that name.
_LEGACY_DOCKER_MOUNT = "/aiformula_ws"


def _workspace_roots():
    roots = []

    env_root = os.environ.get(_ENV_VAR)
    if env_root:
        roots.append(env_root)

    roots.append(_DOCKER_MOUNT)
    roots.append(_LEGACY_DOCKER_MOUNT)

    # This file lives at <workspace_root>/common/common_python/common_python/workspace_paths.py
    here = os.path.dirname(os.path.abspath(__file__))
    roots.append(os.path.abspath(os.path.join(here, "..", "..", "..")))

    # Sibling checkout on this dev machine that may still hold the large mp4/model files.
    roots.append(os.path.expanduser("~/aiformula_ws"))

    seen = set()
    unique = []
    for root in roots:
        if root and root not in seen:
            seen.add(root)
            unique.append(root)
    return unique


def resolve_workspace_asset(path: str) -> str:
    """Return the first existing path for an asset recorded relative to a workspace root.

    Accepts an absolute "/aiformula_machine/..." path (the Docker-mount
    default; "/aiformula_ws/..." from older configs is accepted too) or a
    bare relative path (e.g. "models/traffic_light.pt"), and tries it against
    each candidate workspace root. Falls back to the input unchanged if
    nothing on disk matches, so callers can still report a clear "not found"
    error against the originally requested path.
    """
    if not path or os.path.exists(path):
        return path

    relative = path
    for mount in (_DOCKER_MOUNT, _LEGACY_DOCKER_MOUNT):
        if path.startswith(mount + "/"):
            relative = path[len(mount) + 1:]
            break
    else:
        relative = path.lstrip("/")

    for root in _workspace_roots():
        candidate = os.path.join(root, relative)
        if os.path.exists(candidate):
            return candidate

    return path


def default_workspace_asset(*relative_parts: str) -> str:
    """Best-effort absolute default for a workspace-relative asset.

    Used for `DeclareLaunchArgument`/`declare_parameter` defaults, which are
    computed once at launch/declare time. Tries each candidate workspace root
    and returns the first that exists; otherwise falls back to the Docker
    mount path so behavior inside the container is unchanged.
    """
    for root in _workspace_roots():
        candidate = os.path.join(root, *relative_parts)
        if os.path.exists(candidate):
            return candidate
    return os.path.join(_DOCKER_MOUNT, *relative_parts)
