"""Purely vibe-coded. I cannot endorse nor claim any innocence in the partaking of this activity..."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from itertools import product
from typing import TYPE_CHECKING

import numpy as np

from . import instructions as ins
from .types import Vector3

if TYPE_CHECKING:
    from . import Case


@dataclass(frozen=True)
class Collision:
    drone_a: int
    drone_b: int
    instruction_a: int | None
    instruction_b: int | None
    time_ds: float
    distance_cm: float


@dataclass(frozen=True)
class SkippedInterval:
    drone: int  # Public drone.id, not its list index.
    instruction: int | None  # None means a final stationary hold.
    command: str
    start_ds: int
    end_ds: int
    reason: str


class SkippedCollisionChecksWarning(UserWarning):
    """Compatibility name only; diagnostics now use print(), not warnings.warn()."""


@dataclass(frozen=True)
class FallbackDecision:
    """Conditional clearances with all OTHER paths at their final selections.

    The two minima cover this instruction's entire duration against every
    overlapping checked drone, not just the pair that triggered the failure.
    A joint selection also compares the other affected pairs. A tie on the
    first minimum can therefore be broken by another pair's clearance.
    """

    drone: int
    instruction: int
    selected: str  # "original" or "modified".
    original_clearance_cm: float
    modified_clearance_cm: float


@dataclass(frozen=True)
class RepairReport:
    passes: int
    converted_moves: int
    changed_curves: int
    checked_intervals: int
    clearance_cm: float
    skipped_intervals: tuple[SkippedInterval, ...] = ()
    failed_drones: tuple[int, ...] = ()
    unresolved_collisions: tuple[Collision, ...] = ()
    errors: tuple[str, ...] = ()
    final_scan_completed: bool = True
    # Public drone.id and zero-based index in drone.instructions. Final holds
    # have no authored instruction and are described in unresolved_collisions.
    failed_instructions: tuple[tuple[int, int], ...] = ()
    repaired_instructions: tuple[tuple[int, int], ...] = ()
    # Committed edits on instructions where repair failed. These are ALSO in
    # failed_instructions, and are never included in repaired_instructions.
    partial_instructions: tuple[tuple[int, int], ...] = ()
    fallback_decisions: tuple[FallbackDecision, ...] = ()
    # False for a final comparison component exceeding the exhaustive limit.
    # Even True refers only to original/last-candidate choices, NOT all paths.
    fallback_search_exhaustive: bool = True

    @property
    def fully_checked(self) -> bool:
        """Coverage only: does NOT mean there were no collisions."""
        return self.final_scan_completed and not self.skipped_intervals

    @property
    def success(self) -> bool:
        """All modelled paths checked, no failures, exclusions or collisions."""
        return (
            self.fully_checked
            and not self.failed_drones
            and not self.failed_instructions
            and not self.unresolved_collisions
            and not self.errors
        )


class CollisionCheckResult(list[Collision]):
    """List-compatible result with coverage/error metadata for read-only checks.

    As with any list, bool(result) tests whether there are known collisions;
    use result.success to require a complete, error-free, collision-free check.
    """

    def __init__(
        self,
        collisions=(),
        *,
        skipped_intervals=(),
        errors=(),
        final_scan_completed=True,
    ):
        super().__init__(collisions)
        self.skipped_intervals = tuple(skipped_intervals)
        self.errors = tuple(errors)
        self.final_scan_completed = final_scan_completed

    @property
    def fully_checked(self) -> bool:
        return self.final_scan_completed and not self.skipped_intervals

    @property
    def success(self) -> bool:
        return self.fully_checked and not self and not self.errors


class CollisionRepairError(RuntimeError):
    """Compatibility name; the public fixer no longer raises this exception."""

    def __init__(self, reason: str, collisions: list[Collision]):
        self.collisions = collisions
        super().__init__(reason)


@dataclass
class _Segment:
    drone: int  # Index in case.drones, not necessarily drone.id.
    instruction: int | None  # None means the final stationary hold.
    begin: int
    end: int
    p0: np.ndarray | None
    p2: np.ndarray | None
    control: np.ndarray | None
    editable: bool
    checked: bool = True
    issue: str | None = None  # Local input failure; geometry can still be known.


@dataclass
class _Conflict:
    a: int  # Indices in model.segments.
    b: int
    time: float
    distance: float


@dataclass
class _Model:
    case: Case
    segments: list[_Segment]
    authored: list[list[int]]
    controls: np.ndarray
    reference: np.ndarray
    p0: np.ndarray
    p2: np.ndarray
    frame_ids: np.ndarray
    times: np.ndarray
    u0: np.ndarray
    du: np.ndarray
    lower: np.ndarray
    upper: np.ndarray
    skipped: tuple[SkippedInterval, ...]
    disabled: dict[int, str] = field(default_factory=dict)
    attempted_passes: int = 0
    diagnostics: list[str] = field(default_factory=list)


def _vector(v: Vector3) -> np.ndarray:
    a = np.array((v.x, v.y, v.z), dtype=float)
    if not np.all(np.isfinite(a)):
        raise ValueError("Positions and controls must be finite.")
    return a


def _encoded_vector(v: Vector3, name: str) -> np.ndarray:
    a = _vector(v)
    if np.any(a != np.floor(a)):
        raise ValueError(
            f"{name} must already contain integer centimetres. "
            "The ULEB128 encoder truncates fractional coordinates; "
            "resolve that mismatch before collision repair."
        )
    return a


def _unsupported_endpoint(
    instruction: ins.Instruction, start: np.ndarray | None
) -> tuple[np.ndarray | None, str]:
    """Evaluate an endpoint only; never approximate the unsupported trajectory.

    This deliberately does not trust cached start_pos/end_pos fields. Those
    fields may contain defaults or may have been erased by reset_simulation().
    Endpoint failures are recorded, not silently treated as stationary motion.
    Only unsupported instructions use this exception-handling path.
    """
    if start is None:
        return None, "its starting position is unknown"
    try:
        probe = deepcopy(instruction)
        probe.start_pos = Vector3(start.tolist())
        result = probe.calculate_pos(1.0)
        if result is None or result is NotImplemented:
            return None, "calculate_pos(1) has no implementation"
        return _vector(result), "endpoint evaluated with calculate_pos(1)"
    except Exception as exc:
        return None, f"endpoint evaluation failed ({type(exc).__name__}: {exc})"


class _PathBoundsError(ValueError):
    """A known original trajectory violates the supplied control-hull bounds."""


def _read_drone(
    drone: ins.Drone,
    di: int,
    lower: np.ndarray,
    upper: np.ndarray,
    skip_unsupported: bool,
    *,
    tolerate_bounds: bool = False,
):
    """Read geometry without changing any original instructions or caches."""
    segments: list[_Segment] = []
    skipped: list[SkippedInterval] = []
    boundaries = {0}

    def copy_point(point):
        return None if point is None else point.copy()

    ordered = sorted(drone.instructions, key=lambda inst: inst.flag.value)
    if any(a is not b for a, b in zip(drone.instructions, ordered)):
        raise ValueError(
            f"Drone {drone.id}: top-level instruction order differs from "
            "serialization order. Put lighting inside movement instructions "
            "or resolve the ordering before repair."
        )
    p = _vector(drone.start_pos)
    t = 0
    row, timeline = [], []
    for ii, instruction in enumerate(drone.instructions):
        duration = instruction.duration
        if isinstance(duration, bool) or not isinstance(duration, int) or duration < 0:
            raise ValueError("Instruction durations must be integer deciseconds >= 0.")
        kind = type(instruction)
        issue = None
        checked = p is not None
        reason = "Starting position is unknown after an unsupported instruction."
        if kind in (ins.Move3D, ins.Curve3):
            # Absolute targets restore position knowledge at the END of a
            # command, even if its starting position could not be recovered.
            end = _encoded_vector(instruction.path.target, "Movement target")
            control = (
                _encoded_vector(instruction.path.control, "Curve3 control")
                if kind is ins.Curve3
                else ((p + end) * 0.5 if p is not None else None)
            )
        elif kind is ins.Takeoff:
            height = instruction.path.height
            if not np.isfinite(height) or height < 0 or height != int(height):
                raise ValueError("Takeoff height must be a nonnegative integer.")
            end = p + np.array((0.0, 0.0, height)) if p is not None else None
            control = (p + end) * 0.5 if p is not None else None
        elif kind is ins.Land:
            end = np.array((p[0], p[1], 0.0)) if p is not None else None
            control = (p + end) * 0.5 if p is not None else None
        elif kind in (ins.Calibrate, ins.Colour, ins.ColourGradient):
            end, control = copy_point(p), copy_point(p)
        else:
            if not skip_unsupported:
                raise NotImplementedError(
                    f"Drone {drone.id}, instruction {ii}: {kind.__name__} is not "
                    "a supported quadratic/linear/stationary path."
                )
            checked = False
            end, detail = _unsupported_endpoint(instruction, p)
            control = None  # No invented path, even when both ends are known.
            reason = f"Unsupported {kind.__name__}; {detail}."
        editable = checked and kind in (ins.Move3D, ins.Curve3) and duration > 0
        if checked:
            assert p is not None and end is not None and control is not None
            if duration == 0 and (
                not np.array_equal(p, end) or not np.array_equal(p, control)
            ):
                raise ValueError("A zero-duration movement would teleport the drone.")
            for point in (p, control, end):
                if np.any(point < lower) or np.any(point > upper):
                    issue = (
                        f"Drone {drone.id}, instruction {ii}: endpoint/control "
                        "outside the supplied bounds "
                        "(control-hull test is conservative)."
                    )
                    if not tolerate_bounds:
                        raise _PathBoundsError(issue)
                    break
        else:
            skipped.append(
                SkippedInterval(drone.id, ii, kind.__name__, t, t + duration, reason)
            )
        sid = len(segments)
        segments.append(
            _Segment(
                di,
                ii,
                t,
                t + duration,
                copy_point(p),
                copy_point(end),
                copy_point(control),
                editable,
                checked,
                issue,
            )
        )
        row.append(sid)
        if duration:
            timeline.append(sid)
        t += duration  # ALWAYS preserve skipped instruction durations.
        boundaries.add(t)
        p = copy_point(end)
    if (
        not tolerate_bounds
        and p is not None
        and (np.any(p < lower) or np.any(p > upper))
    ):
        raise _PathBoundsError(f"Drone {drone.id}: position outside bounds.")
    return segments, row, timeline, p, t, skipped, boundaries


def _build(
    case: Case,
    resolution: int,
    bounds: tuple[Vector3, Vector3] | None,
    skip_unsupported: bool = True,
    *,
    tolerate_errors: bool = False,
) -> _Model:
    lower = np.full(3, -np.inf)
    upper = np.full(3, np.inf)
    if bounds is not None:
        lower, upper = _vector(bounds[0]), _vector(bounds[1])
        if np.any(upper < lower):
            raise ValueError("Bounds must satisfy lower <= upper.")
        if np.any(np.ceil(lower) > np.floor(upper)):
            raise ValueError("Bounds contain no integer control coordinate.")

    segments: list[_Segment] = []
    authored: list[list[int]] = []
    moving_timelines: list[list[int]] = []
    final_positions: list[np.ndarray | None] = []
    final_times: list[int] = []
    skipped: list[SkippedInterval] = []
    disabled: dict[int, str] = {}
    boundaries = {0}
    horizon_hint = 1

    for di, drone in enumerate(case.drones):
        try:
            local, row, timeline, p, t, omitted, edges = _read_drone(
                drone,
                di,
                lower,
                upper,
                skip_unsupported,
                tolerate_bounds=tolerate_errors,
            )
        except Exception as exc:
            if not tolerate_errors:
                raise
            disabled[di] = f"{type(exc).__name__}: {exc}"
            # This fallback is for malformed timelines/geometry that cannot
            # be modelled safely. Ordinary bounds failures are handled locally
            # by _read_drone and do NOT disable the rest of the drone.
            local, row, timeline, p, t, omitted, edges = [], [], [], None, 0, [], {0}
            try:
                durations = [i.duration for i in drone.instructions]
                hint = sum(v for v in durations if type(v) is int and v >= 0)
                horizon_hint = max(horizon_hint, hint)
            except Exception:
                pass
        offset = len(segments)
        segments.extend(local)
        authored.append([offset + sid for sid in row])
        moving_timelines.append([offset + sid for sid in timeline])
        final_positions.append(p)
        final_times.append(t)
        skipped.extend(omitted)
        boundaries.update(edges)

    horizon = max(horizon_hint, max(final_times, default=0))
    for di, timeline in enumerate(moving_timelines):
        if final_times[di] < horizon:
            p = final_positions[di]

            def cp():
                return None if p is None else p.copy()

            timeline.append(len(segments))
            issue = None
            if p is not None and (np.any(p < lower) or np.any(p > upper)):
                issue = f"Drone {case.drones[di].id}: final position outside bounds."
                if not tolerate_errors:
                    raise _PathBoundsError(issue)
            segments.append(
                _Segment(
                    di,
                    None,
                    final_times[di],
                    horizon,
                    cp(),
                    cp(),
                    cp(),
                    False,
                    p is not None,
                    issue,
                )
            )
            if p is None:
                reason = (
                    f"Invalid drone model: {disabled[di]}"
                    if di in disabled
                    else "Final position is unknown after an unsupported instruction."
                )
                skipped.append(
                    SkippedInterval(
                        case.drones[di].id,
                        None,
                        "EntireDrone" if di in disabled else "FinalHold",
                        final_times[di],
                        horizon,
                        reason,
                    )
                )
    boundaries.update(range(0, horizon, resolution))
    boundaries.add(horizon)
    times = np.array(sorted(boundaries), dtype=float)
    mids = (times[:-1] + times[1:]) * 0.5
    frame_ids = np.empty((len(mids), len(case.drones)), dtype=int)
    for di, timeline in enumerate(moving_timelines):
        ends = np.array([segments[s].end for s in timeline])
        frame_ids[:, di] = np.array(timeline)[np.searchsorted(ends, mids, side="right")]
    starts = np.array([s.begin for s in segments], dtype=float)
    durations = np.array([s.end - s.begin for s in segments], dtype=float)
    u0 = (times[:-1, None] - starts[frame_ids]) / durations[frame_ids]
    du = np.diff(times)[:, None] / durations[frame_ids]

    def points(field: str) -> np.ndarray:
        # Padding only for vectorized array shapes. Unchecked segments are
        # masked out in _scan BEFORE any pair is submitted to _closest().
        return np.array(
            [
                getattr(s, field) if getattr(s, field) is not None else np.zeros(3)
                for s in segments
            ],
            dtype=float,
        ).reshape(-1, 3)

    controls = points("control")
    return _Model(
        case,
        segments,
        authored,
        controls,
        controls.copy(),
        points("p0"),
        points("p2"),
        frame_ids,
        times,
        u0,
        du,
        lower,
        upper,
        tuple(skipped),
        disabled,
    )


def _warn_skipped(model: _Model) -> None:
    # Do not use warnings.warn: warning filters can promote warnings to errors.
    if model.skipped:
        print(
            f"[collision_fix] Skipped {len(model.skipped)} instruction/hold "
            "interval(s). Excluded paths are NOT checked for collisions."
        )


def _closest(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> tuple[float, float]:
    """Minimize ||a*s*s + b*s + c||, 0 <= s <= 1, numerically."""
    aa, bb = float(a @ a), float(b @ b)
    if aa == 0.0:
        s = float(np.clip(-(b @ c) / bb, 0.0, 1.0)) if bb else 0.0
        return s, float(np.linalg.norm(b * s + c))
    # Half the derivative of squared distance, LOWEST power first.
    coefficients = np.array((b @ c, bb + 2.0 * (a @ c), 3.0 * (a @ b), 2.0 * aa))
    coefficients /= np.max(np.abs(coefficients))
    roots = np.polynomial.polynomial.polyroots(coefficients)
    candidates = [0.0, 1.0]
    for root in roots:
        if abs(root.imag) <= 1e-7 and -1e-10 <= root.real <= 1.0 + 1e-10:
            candidates.append(float(np.clip(root.real, 0.0, 1.0)))
    distances = [float(np.linalg.norm((a * s + b) * s + c)) for s in candidates]
    k = int(np.argmin(distances))
    return candidates[k], distances[k]


def _scan(model: _Model, clearance: float) -> list[_Conflict]:
    """Continuous interval scan, retaining the worst conflict per segment pair."""
    ids = model.frame_ids
    checked = np.array([s.checked for s in model.segments], dtype=bool)[ids]
    p0, p2, ctrl = model.p0[ids], model.p2[ids], model.controls[ids]
    qa = p0 - 2.0 * ctrl + p2
    qb = 2.0 * (ctrl - p0)
    u, h = model.u0[..., None], model.du[..., None]
    c = (qa * u + qb) * u + p0
    b = (2.0 * qa * u + qb) * h
    a = qa * h * h
    # Three local Bezier controls enclose the entire subcurve.
    hull = np.stack((c, c + b * 0.5, c + b + a))
    lo, hi = np.min(hull, axis=0), np.max(hull, axis=0)
    left, right = np.triu_indices(len(model.case.drones), 1)
    worst: dict[tuple[int, int], _Conflict] = {}
    for frame in range(len(model.times) - 1):
        gaps = np.maximum(
            np.maximum(
                lo[frame, left] - hi[frame, right], lo[frame, right] - hi[frame, left]
            ),
            0.0,
        )
        eligible = checked[frame, left] & checked[frame, right]
        candidates = np.flatnonzero(
            eligible & (np.sum(gaps * gaps, axis=1) <= clearance**2)
        )
        for pair in candidates:
            i, j = int(left[pair]), int(right[pair])
            s, distance = _closest(
                a[frame, i] - a[frame, j],
                b[frame, i] - b[frame, j],
                c[frame, i] - c[frame, j],
            )
            if distance >= clearance:  # A boundary equality is acceptable.
                continue
            si, sj = int(ids[frame, i]), int(ids[frame, j])
            key = (si, sj)
            if key not in worst or distance < worst[key].distance:
                t = model.times[frame] + s * (
                    model.times[frame + 1] - model.times[frame]
                )
                worst[key] = _Conflict(si, sj, float(t), distance)
    return sorted(
        worst.values(), key=lambda hit: (hit.distance, hit.time, hit.a, hit.b)
    )


# Limit exponential work. These affect fallback selection only, not trajectory
# correction passes. Large-component search strictly improves a finite score
# and is additionally capped. It never replaces a better incumbent by a seed.
_FALLBACK_EXACT_LIMIT = 8
_FALLBACK_SEARCH_PASSES = 8


@dataclass(frozen=True)
class _FallbackResult:
    decisions: tuple[FallbackDecision, ...] = ()
    exhaustive: bool = True


def _overlap_clearance(
    model: _Model, left: int, right: int, ca: np.ndarray, cb: np.ndarray
) -> float:
    """Minimum separation over the FULL overlap of two supported commands.

    Normalize each path onto the same overlap interval before calling _closest.
    This uses the same quadratic model as _scan without sampling only the last
    collision time, or confusing progress through two different commands.
    """
    a, b = model.segments[left], model.segments[right]
    begin, end = max(a.begin, b.begin), min(a.end, b.end)
    if end <= begin:
        return float("inf")

    def coefficients(segment, control):
        duration = segment.end - segment.begin
        u, h = (begin - segment.begin) / duration, (end - begin) / duration
        qa = segment.p0 - 2.0 * control + segment.p2
        qb = 2.0 * (control - segment.p0)
        return qa * h * h, (2.0 * qa * u + qb) * h, (qa * u + qb) * u + segment.p0

    aa, ab, ac = coefficients(a, ca)
    ba, bb, bc = coefficients(b, cb)
    _, distance = _closest(aa - ba, ab - bb, ac - bc)
    if not np.isfinite(distance):
        raise ValueError("Fallback clearance calculation was not finite.")
    return distance


def _select_fallbacks(
    model: _Model, alternatives: dict[int, np.ndarray]
) -> _FallbackResult:
    """Select valid original/modified combinations, then update model.controls.

    The score is the sorted vector of affected pairwise MINIMUM distances,
    worst first. Maximize it lexicographically; on a complete tie prefer fewer
    modified controls, then smaller total squared control displacement. Values
    are rounded to 1e-8 cm only for ranking, to suppress numerical tie noise.
    Unrelated pairs are excluded so an unrelated unavoidable collision cannot
    hide improvements. Every third-party path overlapping an affected command
    IS included, including stationary and already-frozen paths.

    Small connected components are exhaustive. Larger ones compare the current,
    all-original and all-modified seeds, then do bounded single/pair coordinate
    improvement. They do NOT claim the greatest of all 2**N joint assignments.
    Original/candidate choices are reevaluated after any other path changes.
    """
    if not alternatives:
        return _FallbackResult()
    sids = sorted(alternatives)
    count = len(sids)
    positions = {sid: i for i, sid in enumerate(sids)}
    # Last entry is a zero-state sentinel for fixed (noncandidate) segments.
    states = np.zeros(count + 1, dtype=np.int8)
    weights = np.empty(count)
    for i, sid in enumerate(sids):
        states[i] = not np.array_equal(model.controls[sid], model.reference[sid])
        delta = alternatives[sid] - model.reference[sid]
        weights[i] = float(delta @ delta)

    pairs: set[tuple[int, int]] = set()
    checked = [
        i for i, seg in enumerate(model.segments) if seg.checked and seg.end > seg.begin
    ]
    for sid in sids:
        a = model.segments[sid]
        for other in checked:
            b = model.segments[other]
            if a.drone != b.drone and max(a.begin, b.begin) < min(a.end, b.end):
                pairs.add(tuple(sorted((sid, other))))
    pairs = sorted(pairs)
    left = np.array([positions.get(a, count) for a, _ in pairs], dtype=int)
    right = np.array([positions.get(b, count) for _, b in pairs], dtype=int)
    tables = np.empty((len(pairs), 2, 2), dtype=float)
    adjacent = [set() for _ in sids]
    incident = [set() for _ in sids]
    for row, (a, b) in enumerate(pairs):
        av = (
            (model.reference[a], alternatives[a])
            if a in alternatives
            else (model.controls[a],)
        )
        bv = (
            (model.reference[b], alternatives[b])
            if b in alternatives
            else (model.controls[b],)
        )
        for ia, ca in enumerate(av):
            for ib, cb in enumerate(bv):
                tables[row, ia, ib] = _overlap_clearance(model, a, b, ca, cb)
        if len(av) == 1:
            tables[row, 1, : len(bv)] = tables[row, 0, : len(bv)]
        if len(bv) == 1:
            tables[row, :, 1] = tables[row, :, 0]
        if left[row] < count:
            incident[left[row]].add(row)
        if right[row] < count:
            incident[right[row]].add(row)
        if left[row] < count and right[row] < count:
            adjacent[left[row]].add(int(right[row]))
            adjacent[right[row]].add(int(left[row]))

    def rows_for(block):
        return np.array(sorted(set().union(*(incident[i] for i in block))), dtype=int)

    def distances(rows):
        return tables[rows, states[left[rows]], states[right[rows]]]

    def score(block, rows):
        # Comparing only the changed rows is equivalent to comparing the full
        # sorted profile: all other rows are identical for these alternatives.
        ordered = tuple(np.sort(np.round(distances(rows), 8)))
        return (
            ordered,
            -int(np.sum(states[block])),
            -float(np.dot(states[block], weights[block])),
        )

    def choose(block, rows, exhaustive):
        before = states[block].copy()
        best, best_score = before.copy(), score(block, rows)
        choices = (
            product((0, 1), repeat=len(block))
            if exhaustive
            else (np.zeros(len(block), dtype=int), np.ones(len(block), dtype=int))
        )
        for bits in choices:
            states[block] = bits
            candidate_score = score(block, rows)
            if candidate_score > best_score:
                best, best_score = states[block].copy(), candidate_score
        states[block] = best
        return not np.array_equal(before, best)

    unseen = set(range(count))
    exhaustive = True
    while unseen:
        root = min(unseen)
        component, todo = set(), [root]
        while todo:
            node = todo.pop()
            if node in component:
                continue
            component.add(node)
            todo.extend(adjacent[node] - component)
        unseen.difference_update(component)
        block = np.array(sorted(component), dtype=int)
        rows = rows_for(block)
        if len(block) <= _FALLBACK_EXACT_LIMIT:
            choose(block, rows, True)
            continue
        exhaustive = False
        choose(block, rows, False)  # Current versus all-original/all-modified.
        blocks = [np.array((i,), dtype=int) for i in block]
        blocks += [
            np.array((i, j), dtype=int)
            for i in block
            for j in sorted(adjacent[i])
            if j > i
        ]
        searches = [(part, rows_for(part)) for part in blocks]
        for _ in range(_FALLBACK_SEARCH_PASSES):
            changed = False
            for part, part_rows in searches:
                changed |= choose(part, part_rows, True)
            if not changed:
                break

    decisions = []
    for i, sid in enumerate(sids):
        selected = int(states[i])
        rows = np.array(sorted(incident[i]), dtype=int)
        states[i] = 0
        original = float(np.min(distances(rows))) if len(rows) else float("inf")
        states[i] = 1
        modified = float(np.min(distances(rows))) if len(rows) else float("inf")
        states[i] = selected
        seg = model.segments[sid]
        decisions.append(
            FallbackDecision(
                model.case.drones[seg.drone].id,
                seg.instruction,
                "modified" if selected else "original",
                original,
                modified,
            )
        )
    # Finish all fallible scoring work before changing the in-memory model.
    for i, sid in enumerate(sids):
        model.controls[sid] = alternatives[sid] if states[i] else model.reference[sid]
    return _FallbackResult(tuple(decisions), exhaustive)


def _at(model: _Model, sid: int, t: float) -> tuple[np.ndarray, np.ndarray, float]:
    s = model.segments[sid]
    assert s.checked and s.p0 is not None and s.p2 is not None
    u = float(np.clip((t - s.begin) / (s.end - s.begin), 0.0, 1.0))
    control = model.controls[sid]
    p = (1.0 - u) ** 2 * s.p0 + 2.0 * u * (1.0 - u) * control + u * u * s.p2
    v = 2.0 * ((1.0 - u) * (control - s.p0) + u * (s.p2 - control)) / (s.end - s.begin)
    influence = 2.0 * u * (1.0 - u) if s.editable else 0.0
    return p, v, influence


def _public(model: _Model, hits: list[_Conflict]) -> list[Collision]:
    result = []
    for hit in hits:
        a, b = model.segments[hit.a], model.segments[hit.b]
        result.append(
            Collision(
                model.case.drones[a.drone].id,
                model.case.drones[b.drone].id,
                a.instruction,
                b.instruction,
                hit.time,
                hit.distance,
            )
        )
    return result


def _split_correction(
    normal: np.ndarray,
    separation_step: float,
    wa: float,
    wb: float,
    sharing: str,
    max_control_step: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Split a relative POSITION correction between the two controls.

    ``influence`` minimizes squared control displacement for this constraint.
    ``equal`` shares the position change 50/50 at the conflict time, BEFORE
    integer rounding. A common step cap preserves that sharing. If one
    influence is numerically negligible, use the influence-weighted rule
    rather than dividing by a nearly zero number. No curve timing is changed.
    """
    denominator = wa * wa + wb * wb
    if denominator <= 1e-14:
        return np.zeros(3), np.zeros(3)  # Caller reports insufficient influence.
    if sharing == "equal" and min(wa, wb) > 1e-7:
        da = normal * (0.5 * separation_step / wa)
        db = -normal * (0.5 * separation_step / wb)
    else:
        amount = separation_step / denominator
        da, db = normal * (amount * wa), -normal * (amount * wb)
    largest = max(float(np.linalg.norm(da)), float(np.linalg.norm(db)))
    scale = min(1.0, max_control_step / largest) if largest else 1.0
    return da * scale, db * scale


def _candidate(
    model: _Model, sid: int, delta: np.ndarray, max_control_offset: float
) -> tuple[np.ndarray | None, str | None]:
    """Return a proposed integer control; do not change either drone yet.

    Invalid corrections are rejected rather than clipped to the box. The two
    proposals for a collision are applied atomically, or both are discarded.
    """
    candidate = np.rint(model.controls[sid] + delta)
    if not np.all(np.isfinite(candidate)):
        return None, "The proposed control point is not finite."
    if np.any(candidate < model.lower) or np.any(candidate > model.upper):
        return None, "The proposed control point is outside the supplied bounds."
    if np.linalg.norm(candidate - model.reference[sid]) > max_control_offset + 1e-9:
        return None, "The proposed control point exceeds max_control_offset."
    return candidate, None


@dataclass
class _Prepared:
    row: list[ins.Instruction]
    caches: list[tuple[ins.Instruction, Vector3, Vector3]]
    sim_pos: Vector3
    sim_ref_pos: Vector3
    converted: int
    changed: int


class _InstructionPreparationError(RuntimeError):
    """Internal marker that attributes an export failure to exactly one segment."""

    def __init__(self, sid: int, error: Exception):
        self.sid = sid
        super().__init__(f"{type(error).__name__}: {error}")


def _prepare_instruction(model: _Model, sid: int):
    """Prepare/serialize a COPY of one included instruction, including partial edits."""
    segment = model.segments[sid]
    old = model.case.drones[segment.drone].instructions[segment.instruction]
    assert segment.checked and segment.p0 is not None and segment.p2 is not None
    control = model.controls[sid]
    modified = not np.array_equal(control, model.reference[sid])
    converted = changed = 0
    if type(old) is ins.Move3D and modified:
        new = ins.Curve3(
            Vector3(segment.p2.tolist()), Vector3(control.tolist()), old.duration
        )
        curve_type = new.attributes[0x10]
        new.attributes = deepcopy(old.attributes)
        new.attributes[0x10] = curve_type
        new.instructions = deepcopy(old.instructions)
        new.commands = [
            new.path if cmd is old.path else deepcopy(cmd) for cmd in old.commands
        ]
        if not any(cmd is new.path for cmd in new.commands):
            raise ValueError("Move3D.path is missing from its commands list.")
        converted = 1
    else:
        new = deepcopy(old)
        if type(old) is ins.Curve3 and modified:
            new.path.control = Vector3(control.tolist())
            changed = 1
    new.start_pos = Vector3(segment.p0.tolist())
    new.end_pos = Vector3(segment.p2.tolist())
    if type(new) is ins.Curve3:
        new.path.update(new.start_pos)
    new.to_bytes()
    return new, modified, converted, changed


def _prepare_drone(model: _Model, di: int, frozen: set[int]) -> _Prepared:
    """Stage a mixed row: retained repairs and exact original failed objects."""
    drone = model.case.drones[di]
    row, caches = [], []
    converted = changed = 0
    for sid in model.authored[di]:
        segment = model.segments[sid]
        old = drone.instructions[segment.instruction]
        if not segment.checked or (
            sid in frozen and np.array_equal(model.controls[sid], model.reference[sid])
        ):
            # Untouched failed/skipped originals retain their exact objects and
            # caches. A frozen MODIFIED instruction must be staged/serialized.
            row.append(old)
            continue
        try:
            new, modified, new_converted, new_changed = _prepare_instruction(model, sid)
        except Exception as exc:
            raise _InstructionPreparationError(sid, exc) from exc
        converted += new_converted
        changed += new_changed
        if modified:
            row.append(new)
        else:
            # Existing export-cache behaviour remains for checked, unfailed
            # instructions; updates are deferred until the final commit.
            row.append(old)
            caches.append((old, new.start_pos, new.end_pos))
    return _Prepared(
        row, caches, drone.start_pos.copy(), drone.start_pos.copy(), converted, changed
    )


_SIM_FIELDS = (
    "sim_time",
    "sim_pos",
    "sim_ref_pos",
    "sim_instruction",
    "sim_instruction_start_time",
)


def _apply_prepared(model: _Model, prepared: dict[int, _Prepared]) -> tuple[int, int]:
    """Apply staged rows with rollback protection for an unexpected write error."""
    snapshots = []
    for di, update in prepared.items():
        drone = model.case.drones[di]
        snapshots.append(
            (
                drone,
                drone.instructions[:],
                [(inst, inst.start_pos, inst.end_pos) for inst, _, _ in update.caches],
                tuple(getattr(drone, name) for name in _SIM_FIELDS),
            )
        )
    try:
        for di, update in prepared.items():
            drone = model.case.drones[di]
            for inst, start, end in update.caches:
                inst.start_pos, inst.end_pos = start, end
            drone.instructions[:] = update.row
            drone.sim_time = 0
            drone.sim_pos = update.sim_pos
            drone.sim_ref_pos = update.sim_ref_pos
            drone.sim_instruction = 0
            drone.sim_instruction_start_time = 0
    except Exception:
        for drone, row, caches, state in reversed(snapshots):
            drone.instructions[:] = row
            for inst, start, end in caches:
                inst.start_pos, inst.end_pos = start, end
            for name, value in zip(_SIM_FIELDS, state):
                setattr(drone, name, value)
        raise
    return (
        sum(p.converted for p in prepared.values()),
        sum(p.changed for p in prepared.values()),
    )


def check_collisions(
    case: Case,
    resolution: int = 1,
    avoidance_radius: float = 50.0,
    *,
    safety_margin: float = 0.0,
    bounds: tuple[Vector3, Vector3] | None = None,
    skip_unsupported: bool = True,
) -> CollisionCheckResult:
    """Read-only, list-compatible check; prints ordinary errors instead of raising.

    Returns worst conflicts per included segment pair. Inspect result.success
    or result.fully_checked: an empty list alone does not validate skipped or
    invalid paths. The original show is never changed by this function.
    """
    errors = []
    model = None
    try:
        _validate_settings(resolution, avoidance_radius, safety_margin)
        if not case.drones:
            return CollisionCheckResult()
        model = _build(case, resolution, bounds, skip_unsupported, tolerate_errors=True)
        _warn_skipped(model)
        for di, reason in model.disabled.items():
            message = f"Drone {case.drones[di].id} excluded: {reason}"
            errors.append(message)
            print(f"[collision_fix] ERROR: {message}")
        for segment in model.segments:
            if segment.issue:
                errors.append(segment.issue)
                print(f"[collision_fix] ERROR: {segment.issue}")
        return CollisionCheckResult(
            _public(model, _scan(model, avoidance_radius + safety_margin)),
            skipped_intervals=model.skipped,
            errors=errors,
        )
    except Exception as exc:
        message = f"Collision check stopped ({type(exc).__name__}: {exc})."
        errors.append(message)
        print(f"[collision_fix] ERROR: {message} No complete result is available.")
        return CollisionCheckResult(
            skipped_intervals=model.skipped if model else (),
            errors=errors,
            final_scan_completed=False,
        )


def _validate_settings(resolution: int, radius: float, margin: float) -> None:
    if (
        isinstance(resolution, bool)
        or not isinstance(resolution, int)
        or resolution <= 0
    ):
        raise ValueError("resolution must be a positive integer number of deciseconds.")
    if not np.isfinite(radius) or radius <= 0:
        raise ValueError(
            "avoidance_radius must be a positive finite centre separation in cm."
        )
    if not np.isfinite(margin) or margin < 0:
        raise ValueError("safety_margin must be finite and nonnegative.")
    if not np.isfinite(radius + margin):
        raise ValueError("Combined clearance is too large.")


def _repair(
    model: _Model,
    required: float,
    max_passes: int,
    relaxation: float,
    max_control_step: float,
    max_control_offset: float,
    correction_sharing: str = "influence",
    correction_padding: float = 2.0,
    failure_policy: str = "best_clearance",
) -> RepairReport:
    # Segment IDs, NOT drone IDs. A different command on a partially failed
    # drone can still be adjusted and committed successfully.
    frozen: set[int] = set()
    errors = model.diagnostics
    seen: set[bytes] = set()
    passes = 0

    alternatives: dict[int, np.ndarray] = {}
    fallback_dirty = False
    fallback_result = _FallbackResult()

    def freeze(
        indices: set[int],
        reason: str,
        hits: list[_Conflict] | None = None,
        *,
        force_original: bool = False,
    ) -> bool:
        nonlocal fallback_dirty
        newly_frozen = indices - frozen
        # An unexportable retained partial repair may ALREADY be frozen.
        forced = indices & alternatives.keys() if force_original else set()
        if not newly_frozen and not forced:
            return False
        frozen.update(newly_frozen)
        labels = []
        for sid in sorted(newly_frozen | forced):
            segment = model.segments[sid]
            candidate = model.controls[sid].copy()
            can_select = (
                failure_policy == "best_clearance"
                and not force_original
                and segment.checked
                and segment.issue is None
                and segment.editable
                and segment.instruction is not None
                and not np.array_equal(candidate, model.reference[sid])
                and np.all(np.isfinite(candidate))
                and np.all(candidate == np.rint(candidate))
                and np.all(candidate >= model.lower)
                and np.all(candidate <= model.upper)
                and np.linalg.norm(candidate - model.reference[sid])
                <= max_control_offset + 1e-9
            )
            if can_select:
                # Save the LAST VALID APPLIED candidate, never a rejected
                # out-of-bounds proposal. The original is the second option.
                alternatives[sid] = candidate
            else:
                alternatives.pop(sid, None)
                model.controls[sid] = model.reference[sid].copy()
            segment.editable = False  # Frozen paths remain checked obstacles.
            drone_id = model.case.drones[segment.drone].id
            if segment.instruction is None:
                labels.append(f"drone {drone_id} final hold")
            else:
                labels.append(f"drone {drone_id} instruction {segment.instruction}")
        fallback_dirty = True
        seen.clear()
        details = ""
        if hits:
            c = _public(model, list(hits[:1]))[0]
            details = (
                f" Drone {c.drone_a} / drone {c.drone_b}, "
                f"instructions {c.instruction_a} / {c.instruction_b}, "
                f"t={c.time_ds:.6f} ds, separation={c.distance_cm:.6f} cm."
            )
        action = (
            "Original commands retained"
            if force_original or failure_policy == "original"
            else "Original/last-valid paths will be compared where available"
        )
        message = (
            f"{reason}{details} {action} for {'; '.join(labels)}. "
            "Other instructions remain eligible for repair."
        )
        errors.append(message)
        print(f"[collision_fix] ERROR: {message}")
        return True

    def participants(hits: list[_Conflict]) -> set[int]:
        return {sid for hit in hits for sid in (hit.a, hit.b)}

    def public_instructions(indices: set[int]) -> set[tuple[int, int]]:
        return {
            (
                model.case.drones[model.segments[sid].drone].id,
                model.segments[sid].instruction,
            )
            for sid in indices
            if model.segments[sid].instruction is not None
        }

    for di, reason in model.disabled.items():
        freeze(
            {sid for sid, segment in enumerate(model.segments) if segment.drone == di},
            f"Drone timeline could not be modelled ({reason}). "
            "All of that drone's original instructions are retained and excluded.",
        )
    for sid, segment in enumerate(model.segments):
        if segment.issue:
            freeze({sid}, segment.issue)

    # Each failure freezes an additional segment, or removes an unexportable
    # alternative. Correction passes have a global budget. Binary fallback
    # selection is bounded and does not generate new trajectory controls.
    while True:
        if fallback_dirty:
            fallback_result = _select_fallbacks(model, alternatives)
            fallback_dirty = False
        hits = _scan(model, required)
        actionable = [h for h in hits if h.a not in frozen or h.b not in frozen]
        if not actionable:
            # All remaining collisions involve frozen segments. Prepare the
            # surviving instruction-level repairs, not whole-drone transactions.
            prepared = {}
            preparation_failed = False
            for di in range(len(model.case.drones)):
                if di in model.disabled:
                    continue
                try:
                    update = _prepare_drone(model, di, frozen)
                except _InstructionPreparationError as exc:
                    freeze(
                        {exc.sid},
                        f"Cannot prepare repaired instruction ({exc}).",
                        force_original=True,
                    )
                    preparation_failed = True
                    break
                # Other unexpected staging failures are not safely attributable
                # to a particular instruction. The public wrapper handles them
                # without committing any tentative changes.
                if update.converted or update.changed or update.caches:
                    prepared[di] = update
            if preparation_failed:
                continue  # Restoring one path may invalidate another repair.

            failed_drones = set(model.disabled)
            failed_drones.update(model.segments[sid].drone for sid in frozen)
            failed_instructions = public_instructions(frozen)
            for di in model.disabled:
                failed_instructions.update(
                    (model.case.drones[di].id, ii)
                    for ii in range(len(model.case.drones[di].instructions))
                )
            repaired = {
                sid
                for sid, segment in enumerate(model.segments)
                if sid not in frozen
                and segment.checked
                and segment.instruction is not None
                and not np.array_equal(model.controls[sid], model.reference[sid])
            }
            partial = {
                sid
                for sid in frozen
                if model.segments[sid].instruction is not None
                and model.segments[sid].checked
                and not np.array_equal(model.controls[sid], model.reference[sid])
            }
            converted = sum(p.converted for p in prepared.values())
            changed = sum(p.changed for p in prepared.values())
            remaining = tuple(_public(model, hits))
            if remaining:
                print(
                    f"[collision_fix] {len(remaining)} unresolved collision pair(s) "
                    "remain on selected paths/holds; see report.unresolved_collisions."
                )
            for choice in fallback_result.decisions:
                print(
                    f"[collision_fix] Fallback drone {choice.drone} instruction "
                    f"{choice.instruction}: {choice.selected}; conditional minimum "
                    f"clearance original={choice.original_clearance_cm:.6f} cm, "
                    f"modified={choice.modified_clearance_cm:.6f} cm."
                )
            # Finish all fallible geometry/reporting work before the commit.
            report = RepairReport(
                passes=passes,
                converted_moves=converted,
                changed_curves=changed,
                checked_intervals=len(model.times) - 1,
                clearance_cm=required,
                skipped_intervals=model.skipped,
                failed_drones=tuple(
                    sorted(model.case.drones[di].id for di in failed_drones)
                ),
                unresolved_collisions=remaining,
                errors=tuple(errors),
                final_scan_completed=True,
                failed_instructions=tuple(sorted(failed_instructions)),
                repaired_instructions=tuple(sorted(public_instructions(repaired))),
                partial_instructions=tuple(sorted(public_instructions(partial))),
                fallback_decisions=fallback_result.decisions,
                fallback_search_exhaustive=fallback_result.exhaustive,
            )
            _apply_prepared(model, prepared)
            return report

        if passes >= max_passes:
            freeze(
                participants(actionable),
                "Collision repair reached max_passes.",
                actionable,
            )
            continue
        signature = model.controls.tobytes()
        if signature in seen:
            freeze(
                participants(actionable),
                "Collision repair cycled without convergence.",
                actionable,
            )
            continue
        seen.add(signature)
        passes += 1
        model.attempted_passes = passes
        rolled_back = False
        did_change = False
        for hit in actionable:
            pa, va, wa = _at(model, hit.a, hit.time)
            pb, vb, wb = _at(model, hit.b, hit.time)
            relative = pa - pb
            distance = float(np.linalg.norm(relative))
            if distance >= required:
                continue
            current = _Conflict(hit.a, hit.b, hit.time, distance)
            denominator = wa * wa + wb * wb
            if denominator <= 1e-14:
                freeze(
                    participants([hit]),
                    "Insufficient control-point influence: endpoint, fixed path "
                    "or time too close to an endpoint.",
                    [current],
                )
                rolled_back = True
                break
            if distance > 1e-6:
                normal = relative / distance
            else:
                velocity = va - vb
                normal = np.array((-velocity[1], velocity[0], 0.0))
                norm = float(np.linalg.norm(normal))
                normal = normal / norm if norm > 1e-9 else np.array((1.0, 0.0, 0.0))
            # Padding is local overshoot, NOT extra validated clearance.
            # Enlarge safety_margin to enlarge the whole-show requirement.
            separation_step = relaxation * (required + correction_padding - distance)
            da, db = _split_correction(
                normal, separation_step, wa, wb, correction_sharing, max_control_step
            )
            proposals = []
            failure = None
            for sid, influence, delta in ((hit.a, wa, da), (hit.b, wb, db)):
                if not influence:
                    continue
                candidate, failure = _candidate(model, sid, delta, max_control_offset)
                if failure:
                    break
                proposals.append((sid, candidate))
            if failure is None and not any(
                not np.array_equal(candidate, model.controls[sid])
                for sid, candidate in proposals
            ):
                failure = (
                    "No usable correction remains after integer-centimetre rounding."
                )
            if failure:
                freeze(participants([hit]), failure, [current])
                rolled_back = True
                break
            for sid, candidate in proposals:
                model.controls[sid] = candidate
            fallback_dirty = bool(alternatives)
            did_change = True
        if rolled_back:
            continue
        if not did_change:
            # Re-evaluate before assigning failure to a stale conflict.
            current_hits = [
                h
                for h in _scan(model, required)
                if h.a not in frozen or h.b not in frozen
            ]
            if current_hits:
                freeze(
                    participants(current_hits),
                    "No admissible control change was found.",
                    current_hits,
                )


def fix_collisions(
    case: Case,
    resolution: int = 1,
    avoidance_radius: float = 50.0,
    *,
    safety_margin: float = 5.0,
    max_passes: int = 200,
    relaxation: float = 0.7,
    max_control_step: float = 40.0,
    max_control_offset: float = 600.0,
    bounds: tuple[Vector3, Vector3] | None = None,
    skip_unsupported: bool = True,
    correction_sharing: str = "influence",
    correction_padding: float = 2.0,
    failure_policy: str = "best_clearance",
) -> RepairReport:
    """Best-effort repair. Print ordinary failures; do not propagate exceptions.

    Time is integer deciseconds and distance is centimetres. avoidance_radius
    is minimum CENTRE-TO-CENTRE separation, not an individual drone's radius.
    Required model separation is avoidance_radius + safety_margin. Defaults
    are tuning examples, not validated aircraft limits. Continuous minima
    are checked within each resolution interval and at instruction boundaries.

    Both eligible controls are adjusted in opposite directions. The default
    correction_sharing="influence" retains the previous weighted correction.
    "equal" instead requests equal POSITION displacements at the collision
    time, not equal control displacements. Quantization can make the actual
    shares slightly unequal. At a fixed endpoint or numerically negligible
    influence, use influence weighting so the other drone can do the work.
    Equal sharing can need larger control changes near endpoints; it is not
    guaranteed to converge better than influence weighting.

    correction_padding (cm) replaces the previously hardcoded 2 cm correction
    overshoot. It affects corrections to detected conflicts ONLY. It is not
    added to the whole-show clearance requirement; increase safety_margin for
    that. Larger steps/offset limits permit larger controls, but do not themselves
    require larger separation. Defaults remain unchanged for compatibility.

    Only Move3D/Curve3 controls may be edited. Endpoints, timing and lighting
    are preserved. Failure handling is PER INSTRUCTION. failure_policy defaults
    to "best_clearance": an unresolved instruction offers its ORIGINAL control
    and its LAST ADMISSIBLE APPLIED control as alternatives. It does not retain
    every intermediate attempt. "original" restores the previous unconditional
    rollback behaviour. Invalid new proposals are never offered as alternatives.

    The selector compares continuous pairwise minimum distances over entire
    overlapping commands, including all other checked drones. It maximizes the
    worst distance first, then the next worst, etc. Distances are ranked at
    1e-8 cm precision; complete ties prefer fewer edits, then smaller offsets.
    Up to eight coupled candidate instructions are searched exhaustively;
    larger groups use bounded single/pair coordinate search, not a guarantee
    of the best global combination. Known paths remain checked even if frozen.
    Unsupported/unknown paths remain excluded.

    Insufficient influence, integer rounding stagnation, cycles, bounds/offset
    rejection and max_passes print diagnostics, stop edits to affected commands,
    and select a fallback. max_passes is the GLOBAL correction-pass budget.
    Every fallback and subsequent correction triggers a whole-show recheck;
    fallback choices are reconsidered when other paths change. Other resolved
    commands on the same drone remain eligible for retention. A partial repair
    still appears in failed_instructions/errors, even if its clearance improved.

    A retained modified instruction must pass the normal preparation/serialization
    checks on a copy. An unexportable candidate is removed and its original is
    restored, even if that has worse clearance. Failed originals are not changed,
    including their caches. Successful or partial edits reset that drone's
    simulation state. Malformed entire timelines are excluded. Invalid global
    settings/unexpected failures print an error and leave the whole show unchanged.

    Skipping unsupported commands defaults to True. With False, an unsupported
    command causes its whole drone to be printed/reported as failed and excluded;
    it still does not raise. Excluded commands are not serialized by the fixer.
    Their own implementation/caches are still needed for a later case.save().

    report.success requires no failures, known collisions or unchecked paths.
    report.fully_checked means COVERAGE only. failed_instructions and
    repaired_instructions contain (public drone.id, zero-based instruction index)
    pairs. partial_instructions lists retained edits that are ALSO failures,
    separately from repaired_instructions. fallback_decisions reports conditional
    original/modified minima against the FINAL other paths. failed_drones is a
    compatibility summary, NOT a whole-drone rollback list. Final holds appear
    in unresolved_collisions with instruction=None.
    Inspect errors, skipped_intervals and final_scan_completed as well.
    A normal return is not a collision-free or real-flight safety guarantee.
    KeyboardInterrupt and SystemExit intentionally retain normal behaviour.
    """
    model = None
    required = float("nan")
    try:
        _validate_settings(resolution, avoidance_radius, safety_margin)
        required = float(avoidance_radius + safety_margin)
        if (
            isinstance(max_passes, bool)
            or not isinstance(max_passes, int)
            or max_passes < 0
        ):
            raise ValueError("max_passes must be a nonnegative integer.")
        if not np.isfinite(relaxation) or not 0.0 < relaxation <= 1.0:
            raise ValueError("relaxation must satisfy 0 < relaxation <= 1.")
        if correction_sharing not in ("influence", "equal"):
            raise ValueError('correction_sharing must be "influence" or "equal".')
        if failure_policy not in ("best_clearance", "original"):
            raise ValueError('failure_policy must be "best_clearance" or "original".')
        if not np.isfinite(correction_padding) or correction_padding < 0:
            raise ValueError("correction_padding must be finite and nonnegative.")
        if not np.isfinite(required + correction_padding):
            raise ValueError("Clearance plus correction_padding is too large.")
        for name, value in (
            ("max_control_step", max_control_step),
            ("max_control_offset", max_control_offset),
        ):
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite.")
        if not case.drones:
            return RepairReport(0, 0, 0, 0, required)
        model = _build(case, resolution, bounds, skip_unsupported, tolerate_errors=True)
        _warn_skipped(model)
        return _repair(
            model,
            required,
            max_passes,
            relaxation,
            max_control_step,
            max_control_offset,
            correction_sharing,
            correction_padding,
            failure_policy,
        )
    except Exception as exc:
        # All normal work is staged; _apply_prepared restores originals if an
        # unexpected exception occurs while applying the final prepared rows.
        message = (
            f"Collision repair stopped ({type(exc).__name__}: {exc}). "
            "The original show was not changed."
        )
        print(f"[collision_fix] ERROR: {message}")
        try:
            failed = tuple(d.id for d in case.drones)
            failed_instructions = tuple(
                (d.id, ii) for d in case.drones for ii in range(len(d.instructions))
            )
        except Exception:
            failed, failed_instructions = (), ()
        return RepairReport(
            model.attempted_passes if model else 0,
            0,
            0,
            0,
            required,
            model.skipped if model else (),
            failed,
            (),
            tuple(model.diagnostics if model else ()) + (message,),
            False,
            failed_instructions=failed_instructions,
        )
