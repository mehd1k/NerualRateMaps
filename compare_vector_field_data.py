#!/usr/bin/env python3
"""
Compare vector field data from gazebo_neural_analysis.py and gen_controller.py.

Loads trj/vector_field_data.json (Gazebo) and trj/vector_field_data_synthesized.json
(synthesized), overlays their sample positions, and compares u / neural_rate at a
given position.
"""

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np


DEFAULT_GAZEBO_JSON = "trj/vector_field_data.json"
DEFAULT_SYNTH_JSON = "trj/vector_field_data_synthesized.json"


def load_vector_field_data(json_path: str) -> List[Dict[str, Any]]:
    """Load vector field entries from a JSON file."""
    path = Path(json_path)
    if not path.exists():
        raise FileNotFoundError(f"JSON file not found: {path}")

    with open(path, "r") as f:
        data = json.load(f)

    if not isinstance(data, list):
        raise ValueError(f"Expected a list of entries in {path}")

    return data


def _position_array(entry: Dict[str, Any]) -> np.ndarray:
    return np.asarray(entry["position"], dtype=float)


def find_entry_at_position(
    data: List[Dict[str, Any]],
    position: Tuple[float, float],
    tolerance: float = 0.02,
) -> Optional[Dict[str, Any]]:
    """
    Find the entry whose position is closest to the query, within tolerance.

    Returns None if the closest point is farther than tolerance.
    """
    if not data:
        return None

    query = np.asarray(position, dtype=float)
    distances = [np.linalg.norm(_position_array(entry) - query) for entry in data]
    best_idx = int(np.argmin(distances))

    if distances[best_idx] > tolerance:
        return None

    return data[best_idx]


def compare_at_position(
    position: Tuple[float, float],
    gazebo_data: List[Dict[str, Any]],
    synth_data: List[Dict[str, Any]],
    tolerance: float = 0.02,
) -> Dict[str, Any]:
    """
    Compare u and neural_rate at a given position in both datasets.

    Args:
        position: Query position (x, y).
        gazebo_data: Entries from vector_field_data.json.
        synth_data: Entries from vector_field_data_synthesized.json.
        tolerance: Maximum distance for matching a stored sample position.

    Returns:
        Dictionary with matched entries, absolute/relative differences, and summary stats.
    """
    gazebo_entry = find_entry_at_position(gazebo_data, position, tolerance)
    synth_entry = find_entry_at_position(synth_data, position, tolerance)

    if gazebo_entry is None:
        raise ValueError(
            f"No Gazebo entry within tolerance {tolerance} of position {position}"
        )
    if synth_entry is None:
        raise ValueError(
            f"No synthesized entry within tolerance {tolerance} of position {position}"
        )

    gazebo_pos = _position_array(gazebo_entry)
    synth_pos = _position_array(synth_entry)
    gazebo_u = np.asarray(gazebo_entry["u"], dtype=float)
    synth_u = np.asarray(synth_entry["u"], dtype=float)
    gazebo_nr = np.asarray(gazebo_entry["neural_rate"], dtype=float)
    synth_nr = np.asarray(synth_entry["neural_rate"], dtype=float)

    if gazebo_nr.shape != synth_nr.shape:
        raise ValueError(
            f"Neural rate length mismatch: gazebo={gazebo_nr.size}, synth={synth_nr.size}"
        )

    u_diff = gazebo_u - synth_u
    nr_diff = gazebo_nr - synth_nr

    return {
        "query_position": list(position),
        "gazebo_position": gazebo_pos.tolist(),
        "synth_position": synth_pos.tolist(),
        "position_distance": {
            "gazebo": float(np.linalg.norm(gazebo_pos - np.asarray(position))),
            "synth": float(np.linalg.norm(synth_pos - np.asarray(position))),
            "gazebo_vs_synth": float(np.linalg.norm(gazebo_pos - synth_pos)),
        },
        "gazebo_u": gazebo_u.tolist(),
        "synth_u": synth_u.tolist(),
        "u_diff": u_diff.tolist(),
        "u_l2": float(np.linalg.norm(u_diff)),
        "u_max_abs": float(np.max(np.abs(u_diff))),
        "gazebo_neural_rate": gazebo_nr.tolist(),
        "synth_neural_rate": synth_nr.tolist(),
        "neural_rate_diff": nr_diff.tolist(),
        "neural_rate_l2": float(np.linalg.norm(nr_diff)),
        "neural_rate_max_abs": float(np.max(np.abs(nr_diff))),
        "neural_rate_mean_abs": float(np.mean(np.abs(nr_diff))),
    }


def print_comparison(result: Dict[str, Any]) -> None:
    """Pretty-print the output of compare_at_position."""
    print(f"Query position: {result['query_position']}")
    print(f"Matched Gazebo position: {result['gazebo_position']}")
    print(f"Matched synthesized position: {result['synth_position']}")
    print(
        "Position distances: "
        f"gazebo={result['position_distance']['gazebo']:.6f}, "
        f"synth={result['position_distance']['synth']:.6f}, "
        f"gazebo_vs_synth={result['position_distance']['gazebo_vs_synth']:.6f}"
    )
    print(f"u (gazebo):      {result['gazebo_u']}")
    print(f"u (synthesized): {result['synth_u']}")
    print(f"u diff:          {result['u_diff']}")
    print(f"u L2={result['u_l2']:.6f}, max|diff|={result['u_max_abs']:.6f}")
    print(
        f"neural_rate L2={result['neural_rate_l2']:.6f}, "
        f"max|diff|={result['neural_rate_max_abs']:.6f}, "
        f"mean|diff|={result['neural_rate_mean_abs']:.6f}"
    )


def _extract_positions_and_u(
    data: List[Dict[str, Any]],
) -> Tuple[np.ndarray, np.ndarray]:
    positions = np.array([entry["position"] for entry in data], dtype=float)
    u_vectors = np.array([entry["u"] for entry in data], dtype=float)
    return positions, u_vectors


def _shared_quiver_scale(
    *u_arrays: np.ndarray,
    target_arrow_length: float = 0.04,
) -> float:
    max_mag = 0.0
    for u in u_arrays:
        if len(u) == 0:
            continue
        max_mag = max(max_mag, float(np.max(np.linalg.norm(u, axis=1))))

    if max_mag == 0.0:
        return 1.0
    return max_mag / target_arrow_length


def plot_positions_overlay(
    gazebo_data: List[Dict[str, Any]],
    synth_data: List[Dict[str, Any]],
    output_path: Optional[str] = None,
    show: bool = True,
) -> Tuple[plt.Figure, plt.Axes]:
    """
    Plot sample positions and control vectors u from both datasets on the same axes.

    Returns matplotlib figure and axes.
    """
    gazebo_pos, gazebo_u = _extract_positions_and_u(gazebo_data)
    synth_pos, synth_u = _extract_positions_and_u(synth_data)
    quiver_scale = _shared_quiver_scale(gazebo_u, synth_u)

    fig, ax = plt.subplots(figsize=(10, 8))

    ax.quiver(
        gazebo_pos[:, 0],
        gazebo_pos[:, 1],
        gazebo_u[:, 0],
        gazebo_u[:, 1],
        angles="xy",
        scale_units="xy",
        scale=quiver_scale,
        color="tab:blue",
        width=0.004,
        headwidth=4,
        headlength=5,
        alpha=0.85,
        label=f"Gazebo u ({len(gazebo_pos)})",
    )
    ax.quiver(
        synth_pos[:, 0],
        synth_pos[:, 1],
        synth_u[:, 0],
        synth_u[:, 1],
        angles="xy",
        scale_units="xy",
        scale=quiver_scale,
        color="tab:orange",
        width=0.003,
        headwidth=4,
        headlength=5,
        alpha=0.85,
        label=f"Synthesized u ({len(synth_pos)})",
    )
    ax.scatter(
        gazebo_pos[:, 0],
        gazebo_pos[:, 1],
        c="tab:blue",
        s=30,
        alpha=0.9,
        marker="o",
        zorder=3,
    )
    ax.scatter(
        synth_pos[:, 0],
        synth_pos[:, 1],
        c="tab:orange",
        s=30,
        alpha=0.9,
        marker="x",
        zorder=3,
    )

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Vector field overlay (positions and u)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    if output_path:
        fig.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"Saved vector field overlay plot to {output_path}")

    if show:
        plt.show()

    return fig, ax


def summarize_position_sets(
    gazebo_data: List[Dict[str, Any]],
    synth_data: List[Dict[str, Any]],
) -> Dict[str, Any]:
    """Summarize how well the two position sets align."""
    gazebo_pos = np.array([entry["position"] for entry in gazebo_data])
    synth_pos = np.array([entry["position"] for entry in synth_data])

    pair_distances = []
    for g_pos in gazebo_pos:
        dists = np.linalg.norm(synth_pos - g_pos, axis=1)
        pair_distances.append(float(np.min(dists)))

    return {
        "gazebo_count": len(gazebo_data),
        "synth_count": len(synth_data),
        "min_pair_distance": float(np.min(pair_distances)) if pair_distances else None,
        "max_pair_distance": float(np.max(pair_distances)) if pair_distances else None,
        "mean_pair_distance": float(np.mean(pair_distances)) if pair_distances else None,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare Gazebo and synthesized vector field JSON data."
    )
    parser.add_argument(
        "--gazebo-json",
        type=str,
        default=DEFAULT_GAZEBO_JSON,
        help="Path to vector_field_data.json",
    )
    parser.add_argument(
        "--synth-json",
        type=str,
        default=DEFAULT_SYNTH_JSON,
        help="Path to vector_field_data_synthesized.json",
    )
    parser.add_argument(
        "--position",
        type=float,
        nargs=2,
        metavar=("X", "Y"),
        default=[0.16, 0.99],
        help="Position to compare u and neural_rate",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.02,
        help="Position matching tolerance",
    )
    parser.add_argument(
        "--plot-output",
        type=str,
        default="trj/vector_field_positions_overlay.png",
        help="Path to save the position overlay plot",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open the matplotlib window",
    )
    args = parser.parse_args()

    gazebo_data = load_vector_field_data(args.gazebo_json)
    synth_data = load_vector_field_data(args.synth_json)

    summary = summarize_position_sets(gazebo_data, synth_data)
    print(
        f"Loaded {summary['gazebo_count']} Gazebo points and "
        f"{summary['synth_count']} synthesized points."
    )
    print(
        "Nearest-neighbor position distances (Gazebo -> synthesized): "
        f"min={summary['min_pair_distance']:.6f}, "
        f"mean={summary['mean_pair_distance']:.6f}, "
        f"max={summary['max_pair_distance']:.6f}"
    )

    plot_positions_overlay(
        gazebo_data,
        synth_data,
        output_path=args.plot_output,
        show=not args.no_show,
    )

    if args.position is not None:
        result = compare_at_position(
            tuple(args.position),
            gazebo_data,
            synth_data,
            tolerance=args.tolerance,
        )
        print()
        print_comparison(result)


if __name__ == "__main__":
    main()
