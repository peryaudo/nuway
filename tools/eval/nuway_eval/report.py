"""``results.csv`` (the schema of M1 §3.10) and ``report.md`` of a run directory.

Columns are stable across milestones and ``compare_runs.py`` joins on the
first four (the protocol key). Paths in the report are relative to the run
directory so it can be copied or archived whole.
"""

from __future__ import annotations

import csv
import statistics
import subprocess
import sys
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any

from nuway_eval.driving_score import (
    PENALTY_KINDS,
    InfractionCounts,
    ScoringConfig,
    driving_score,
)

KEY_COLUMNS = ("route_id", "town", "weather", "seed")
RESULT_COLUMNS = (
    *KEY_COLUMNS,
    "profile",
    "git_sha",
    "run_id",
    "status",
    "route_length_m",
    "completion",
    "driving_score",
    "n_collision_pedestrian",
    "n_collision_vehicle",
    "n_collision_layout",
    "n_red_light",
    "n_stop_sign",
    "outside_lanes_frac",
    "n_min_speed",
    "min_speed_pct",
    "route_deviation",
    "blocked",
    "timeout",
    "n_safety_interventions",
    "n_mpc_failures",
    "planner_p50_ms",
    "planner_p99_ms",
    "mpc_p50_ms",
    "mpc_p99_ms",
    "n_tick_timeouts",
    "first_timeout_tick",
    "degraded_sources",
    "non_deterministic",
    "crashed",
    "n_camera_drops",
    "sim_time_s",
    "wall_time_s",
    "bag_path",
)
# The incident vocabulary of docs/02 §8.2: an infraction kind, a route ending,
# or one of the stack events below; a sheet and the row that reports it share
# the name.
STACK_EVENT_KINDS = ("safety_intervention", "mpc_failure", "tick_timeout")
# Sheets rendered per kind per route at most; the rest are counted, not drawn
# (a flapping safety layer intervened on 7396 ticks of one smoke drive).
MAX_INCIDENTS_PER_KIND = 20
REPO_ROOT = Path(__file__).resolve().parents[3]
RENDER_BAG = REPO_ROOT / "tools/viz/render_bag.py"
# Statuses that end a route for good; anything else is rerun by --resume.
FINAL_STATUSES = ("completed", "blocked", "timeout", "route_deviation")


@dataclass(slots=True)
class RouteResult:
    """One row of ``results.csv``."""

    route_id: str
    town: str
    weather: str
    seed: int
    profile: str = ""
    git_sha: str = ""
    run_id: str = ""
    status: str = "not_run"
    route_length_m: float = 0.0
    completion: float = 0.0
    driving_score: float = 0.0
    n_collision_pedestrian: int = 0
    n_collision_vehicle: int = 0
    n_collision_layout: int = 0
    n_red_light: int = 0
    n_stop_sign: int = 0
    outside_lanes_frac: float = 0.0
    n_min_speed: int = 0
    min_speed_pct: float = (
        100.0  # ego speed as % of the traffic's (LB2 min-speed value)
    )
    route_deviation: bool = False
    blocked: bool = False
    timeout: bool = False
    n_safety_interventions: int = 0
    n_mpc_failures: int = 0
    planner_p50_ms: float = 0.0
    planner_p99_ms: float = 0.0
    mpc_p50_ms: float = 0.0
    mpc_p99_ms: float = 0.0
    n_tick_timeouts: int = 0
    first_timeout_tick: int = -1
    degraded_sources: str = ""
    non_deterministic: bool = False
    crashed: bool = False
    n_camera_drops: int = 0
    sim_time_s: float = 0.0
    wall_time_s: float = 0.0
    bag_path: str = ""
    # Not a CSV column: every (tick, kind) event the route produced, for §8.2.
    incidents: list[tuple[int, str]] = field(default_factory=list)

    @property
    def key(self) -> tuple[str, str, str, int]:
        """The protocol key."""
        return (self.route_id, self.town, self.weather, self.seed)

    @property
    def run_dir_name(self) -> str:
        """``<route>_<weather>_<seed>`` (docs/02 §8)."""
        return f"{self.route_id}_{self.weather}_{self.seed}"

    def apply_counts(self, counts: InfractionCounts, config: ScoringConfig) -> None:
        """Copy the infraction columns and score the row."""
        for kind in PENALTY_KINDS:
            setattr(self, f"n_{kind}", counts.count(kind))
        self.outside_lanes_frac = counts.outside_lanes_frac
        self.min_speed_pct = counts.min_speed_pct
        self.route_deviation = counts.route_deviation
        self.incidents = sorted({*self.incidents, *counts.incidents})
        self.blocked = counts.blocked
        self.timeout = counts.timeout
        self.driving_score = driving_score(self.completion, counts, config)

    def row(self) -> dict[str, Any]:
        """CSV row (booleans as 0/1, floats rounded for stable diffs)."""
        out: dict[str, Any] = {}
        for c in RESULT_COLUMNS:
            v = getattr(self, c)
            if isinstance(v, bool):
                v = int(v)
            elif isinstance(v, float):
                v = round(v, 4)
            out[c] = v
        return out

    @classmethod
    def from_row(cls, row: dict[str, str]) -> RouteResult:
        """Inverse of :meth:`row`."""
        kwargs: dict[str, Any] = {}
        for f in fields(cls):
            raw = row.get(f.name)
            if raw is None:
                continue
            if f.type in ("bool", bool):
                kwargs[f.name] = raw not in ("0", "", "False", "false")
            elif f.type in ("int", int):
                kwargs[f.name] = int(float(raw)) if raw != "" else 0
            elif f.type in ("float", float):
                kwargs[f.name] = float(raw) if raw != "" else 0.0
            else:
                kwargs[f.name] = raw
        return cls(**kwargs)


def percentile(values: Sequence[float], q: float) -> float:
    """Nearest-rank percentile (q in [0, 100]); 0 for no values."""
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, round(q / 100.0 * (len(ordered) - 1))))
    return float(ordered[idx])


def write_results(results: Iterable[RouteResult], out_dir: Path) -> Path:
    """Write ``results.csv``."""
    path = out_dir / "results.csv"
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=RESULT_COLUMNS)
        w.writeheader()
        for r in results:
            w.writerow(r.row())
    return path


def read_results(path: Path) -> list[RouteResult]:
    """Read ``results.csv``; an absent file is an empty run."""
    if not path.exists():
        return []
    with path.open(newline="") as f:
        return [RouteResult.from_row(row) for row in csv.DictReader(f)]


def _mean(values: Sequence[float]) -> float:
    return float(statistics.fmean(values)) if values else 0.0


def _fmt(v: object) -> str:
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return f"{v:.3f}"
    return str(v)


def _table(header: Sequence[str], rows: Iterable[Sequence[object]]) -> list[str]:
    lines = [
        "| " + " | ".join(header) + " |",
        "|" + "|".join("---" for _ in header) + "|",
    ]
    lines += ["| " + " | ".join(_fmt(c) for c in row) + " |" for row in rows]
    return lines


def coalesce_incidents(
    events: Sequence[tuple[int, str]],
    window_after: int,
    cap: int = MAX_INCIDENTS_PER_KIND,
) -> tuple[list[tuple[int, str]], dict[str, int]]:
    """Merge events of one kind that fall inside the previous sheet's window; cap per kind.

    Returns the events to render and, per kind, how many were dropped, so the
    report can say "and 312 more" instead of rendering a sheet per tick of a
    flapping safety layer.
    """
    kept: list[tuple[int, str]] = []
    dropped: dict[str, int] = {}
    last: dict[str, int] = {}
    count: dict[str, int] = {}
    for tick, kind in sorted(events):
        merged = kind in last and tick <= last[kind] + window_after
        if merged or count.get(kind, 0) >= cap:
            dropped[kind] = dropped.get(kind, 0) + 1
            continue
        last[kind] = tick
        count[kind] = count.get(kind, 0) + 1
        kept.append((tick, kind))
    return kept, dropped


RunCommand = Callable[[list[str]], object]


def _run_render_bag(cmd: list[str]) -> object:
    return subprocess.run(cmd, check=False, cwd=REPO_ROOT)  # our own script, fixed argv


def render_incidents(
    out_dir: Path,
    result: RouteResult,
    window: tuple[int, int],
    run: RunCommand = _run_render_bag,
) -> list[Path]:
    """Render one sheet per coalesced incident of a route with ``render_bag.py``.

    ``render_bag.py`` is a subprocess rather than an import: it lives in
    ``tools/viz`` and must keep running without this package or ROS.
    ``incidents/summary.txt`` records what was rendered and what was dropped.
    """
    if not result.bag_path or not result.incidents:
        return []
    route_dir = out_dir / result.run_dir_name
    bag = out_dir / result.bag_path
    kept, dropped = coalesce_incidents(result.incidents, window[1])
    cmd = [sys.executable, str(RENDER_BAG), "--bag", str(bag), "--out", str(route_dir)]
    cmd += ["--window", f"{window[0]}:{window[1]}"]
    for tick, kind in kept:
        cmd += ["--incident", f"{tick}:{kind}"]
    run(cmd)
    inc_dir = route_dir / "incidents"
    inc_dir.mkdir(parents=True, exist_ok=True)
    kinds = sorted({k for _, k in result.incidents})
    lines = [
        f"{kind}: {sum(1 for _, k in kept if k == kind)} rendered, {dropped.get(kind, 0)} more"
        for kind in kinds
    ]
    (inc_dir / "summary.txt").write_text("\n".join(lines) + "\n")
    return [inc_dir / f"{tick:06d}_{kind}.png" for tick, kind in kept]


def render_full(
    out_dir: Path, result: RouteResult, stride: int, run: RunCommand = _run_render_bag
) -> None:
    """Render the whole route at ``stride`` into ``frames/`` and ``sheets/``."""
    if not result.bag_path:
        return
    route_dir = out_dir / result.run_dir_name
    bag = out_dir / result.bag_path
    run(
        [
            sys.executable,
            str(RENDER_BAG),
            "--bag",
            str(bag),
            "--out",
            str(route_dir),
            "--stride",
            str(max(1, stride)),
        ]
    )


def incident_links(run_dir: Path, result: RouteResult) -> list[tuple[int, str, str]]:
    """``(tick, kind, relative path)`` of every rendered incident sheet of a route."""
    inc_dir = run_dir / result.run_dir_name / "incidents"
    if not inc_dir.is_dir():
        return []
    out = []
    for png in sorted(inc_dir.glob("*.png")):
        stem = png.stem
        tick_str, _, kind = stem.partition("_")
        if tick_str.isdigit():
            out.append((int(tick_str), kind, str(png.relative_to(run_dir))))
    return out


def _score_section(
    results: Sequence[RouteResult], scored: Sequence[RouteResult]
) -> list[str]:
    lines = ["## Driving score", ""]
    lines.append(
        f"Overall: **{_mean([r.driving_score for r in scored]):.1f}** over {len(scored)} runs "
        f"(completion mean {_mean([r.completion for r in scored]):.3f})."
    )
    lines.append("")
    rows = []
    for town in sorted({r.town for r in results}):
        rs = [r for r in scored if r.town == town]
        rows.append(
            (
                town,
                len(rs),
                _mean([r.driving_score for r in rs]),
                _mean([r.completion for r in rs]),
                sum(1 for r in rs if r.status == "completed"),
            )
        )
    lines += _table(("town", "runs", "driving_score", "completion", "completed"), rows)
    return [*lines, ""]


def _routes_section(results: Sequence[RouteResult], out_dir: Path) -> list[str]:
    cols = (
        "route_id",
        "weather",
        "seed",
        "status",
        "completion",
        "driving_score",
        "sim_time_s",
        "n_tick_timeouts",
        "bag",
        "incidents",
    )
    rows = [
        (
            r.route_id,
            r.weather,
            r.seed,
            r.status + (" (crashed)" if r.crashed else ""),
            r.completion,
            r.driving_score,
            r.sim_time_s,
            r.n_tick_timeouts,
            f"[mcap]({r.bag_path})" if r.bag_path else "",
            _incident_cell(out_dir, r),
        )
        for r in results
    ]
    return ["## Routes", "", *_table(cols, rows), ""]


def _incident_cell(out_dir: Path, r: RouteResult) -> str:
    """``[N sheets](#anchor)`` linking the row to its incident list."""
    n = len(incident_links(out_dir, r))
    if n == 0:
        return "-"
    return f"[{n} sheets](#{r.run_dir_name.lower()})"


def _infraction_section(
    results: Sequence[RouteResult], scored: Sequence[RouteResult]
) -> list[str]:
    hist = []
    for kind in (*PENALTY_KINDS, "route_deviation", "blocked", "timeout"):
        if kind in PENALTY_KINDS:
            n = sum(getattr(r, f"n_{kind}") for r in results)
        else:
            n = sum(1 for r in results if getattr(r, kind))
        hist.append((kind, n))
    return [
        "## Infractions",
        "",
        *_table(("kind", "count"), hist),
        "",
        f"Outside-lanes fraction mean {_mean([r.outside_lanes_frac for r in scored]):.4f}; "
        f"min-speed value mean {_mean([r.min_speed_pct for r in scored]):.1f} % "
        "(ego speed over the traffic's, 100 = no checkpoint failed).",
        "",
    ]


def _diag_section(
    results: Sequence[RouteResult], scored: Sequence[RouteResult]
) -> list[str]:
    header = (
        "route_id",
        "weather",
        "planner_p50_ms",
        "planner_p99_ms",
        "mpc_p50_ms",
        "mpc_p99_ms",
        "safety_interventions",
        "mpc_failures",
        "degraded",
    )
    rows = [
        (
            r.route_id,
            r.weather,
            r.planner_p50_ms,
            r.planner_p99_ms,
            r.mpc_p50_ms,
            r.mpc_p99_ms,
            r.n_safety_interventions,
            r.n_mpc_failures,
            r.degraded_sources or "-",
        )
        for r in results
    ]
    worst_planner = percentile([r.planner_p99_ms for r in scored], 100)
    worst_mpc = percentile([r.mpc_p99_ms for r in scored], 100)
    return [
        "## Diagnostics",
        "",
        *_table(header, rows),
        "",
        f"Planner cycle p99 over the run: {worst_planner:.1f} ms (budget 15 ms); "
        f"MPC solve p99: {worst_mpc:.2f} ms (budget 3 ms).",
        "",
    ]


def _incident_section(results: Sequence[RouteResult], out_dir: Path) -> list[str]:
    lines = ["## Incidents", ""]
    any_incident = False
    for r in results:
        links = incident_links(out_dir, r)
        if not links:
            continue
        any_incident = True
        lines += [f"### {r.run_dir_name}", ""]
        lines += [
            f"- tick {tick} `{kind}`: [sheet]({path})" for tick, kind, path in links
        ]
        summary = out_dir / r.run_dir_name / "incidents" / "summary.txt"
        if summary.is_file():
            lines += ["", *(f"- {line}" for line in summary.read_text().splitlines())]
        lines.append("")
    if not any_incident:
        lines += ["No incident sheets rendered.", ""]
    return lines


def write_report(
    results: Sequence[RouteResult], out_dir: Path, config: ScoringConfig
) -> Path:
    """Write ``report.md``: scores per town and overall, infractions, diag, links."""
    lines = [f"# Route run report ({config.name})", ""]
    if results:
        r0 = results[0]
        lines.append(
            f"Run `{r0.run_id}`, profile `{r0.profile}`, commit `{r0.git_sha}`: "
            f"{len(results)} route runs."
        )
        lines.append("")
    scored = [r for r in results if not r.crashed and r.status != "not_run"]
    lines += _score_section(results, scored)
    lines += _routes_section(results, out_dir)
    lines += _infraction_section(results, scored)
    lines += _diag_section(results, scored)
    lines += _incident_section(results, out_dir)
    path = out_dir / "report.md"
    path.write_text("\n".join(lines))
    return path
