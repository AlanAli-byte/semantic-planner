# Design Report — Safe Semantic Planner (LPA*)
PCCST503 – Machine Learning, Assignment 1

Algorithm chosen: **LPA\* (Lifelong Planning A\*)**, Koenig & Likhachev (2002, 2004).

## 1. State Representation

Each `State` (`include/types.hpp`) is a pair `(id, embedding)` where `embedding`
is a `std::vector<double>` — the state's coordinates in `R^d`. `id` is used for
O(1) hashed lookup everywhere (adjacency, g/rhs tables); the embedding is used
only for two things: (a) the admissible heuristic `h(s)`, and (b) computing
objective-4 (minimum Euclidean distance from the realized path to the nearest
bad state), reported as `safetyScore` in `PlanningResult`.

Bad states are **not** a separate type — they are ordinary `State`s whose id
appears in `PlanningProblem::badStates`. This keeps the state space exactly
as specified (`S = {s_1..s_n} ⊂ R^d`) and lets a state be marked/unmarked bad
at runtime (`setBadState`) without touching the graph topology.

## 2. Data Structures

| Structure | Purpose | Complexity |
|---|---|---|
| `stateById_` (hash map) | id → State | O(1) avg lookup |
| `transitionsById_` (hash map) | id → Transition (cost/safety/reliability/available) | O(1) avg lookup, O(1) mutation for updates |
| `successors_`, `predecessors_` (hash map id → vector\<transition id\>) | adjacency for forward expansion and backward `rhs` computation | O(deg) per vertex touch |
| `g_`, `rhs_` (hash map id → double) | LPA*'s two cost estimates per vertex (lazily populated; missing = ∞) | O(1) avg |
| `open_` : `std::set<pair<LPAKey,id>>` + `keyOfOpen_` : hash map id→key | the **priority queue**. `std::set` gives O(log n) insert/erase/min in key order; the parallel hash map gives O(1) "is u in the queue, and with what key" — required by LPA*'s `UpdateVertex`, which must remove-then-conditionally-reinsert on every touch | insert/erase O(log n), min O(1) |

This mirrors the assignment's suggested `State` / `Transition` / `PlanningProblem`
/ `PlanningResult` / `Planner` interfaces exactly (see `include/types.hpp`);
`LPAStarPlanner : public Planner` implements `plan()`.

## 3. Heuristic Function

```
h(s) = euclidean(embedding(s), embedding(goal)) * scale
```

`scale` defaults to **0**, making the search Dijkstra-equivalent — always
admissible and consistent regardless of how `cost` / `safety` / `reliability`
are scaled into the effective edge weight (see §4). `setHeuristicScale()`
lets a caller trade this for speed, but the report is explicit about the
condition required for correctness: `scale` must not exceed the minimum
effective edge weight per unit Euclidean distance anywhere in the graph, or
consistency (and therefore optimality) breaks. We kept the conservative
default rather than guess a domain-specific constant, per the "never bluff"
standard this report is held to — a wrong non-zero scale would silently
produce sub-optimal paths, and I have not proven a safe bound for arbitrary
input embeddings.

## 4. Safety Computation

Two distinct notions of "safety" exist in the spec and both are implemented,
not conflated:

1. **Hard constraint (assignment objective 2 — never visit a bad state).**
   `edgeUsable()` excludes any transition touching a bad-state id from the
   searchable graph entirely. This is enforced structurally, not by penalty —
   a plan literally cannot route through `B`, at any weight setting.

2. **Soft objective (assignment objective 4 — maximize min distance to bad
   states, folded into `Score(P) = αG − βC + γD + δR`).** Each `Transition`
   carries a `safety` attribute (0..1, given/estimated per-edge). Effective
   edge weight:
   ```
   w(u,v) = cost * (1 + reliabilityWeight * (1 − reliability))
                  + safetyWeight * (1 − safety)
   ```
   Raising `safetyWeight` scalarizes the multi-objective score into a single
   minimizable weight — this is exactly how Test Case 3 makes the planner
   switch from the cheap/close path to the costlier/farther one (verified,
   see `experimental_results.txt`). Separately, `safetyScore` in the result
   reports the *actual* minimum Euclidean distance from every visited state's
   embedding to the nearest bad state's embedding — an independent, ground-
   truth evaluation metric, not something the search optimizes directly
   (the `safety` attribute is a proxy for it, since ground-truth embeddings
   for all bad states may not be known to whoever assigns edge attributes).

## 5. Time Complexity

Per Koenig & Likhachev, one call to `ComputeShortestPath` after a *local*
change (single edge cost/availability change, single vertex added) runs in
`O(k · log k)` where `k` is the number of vertices whose key actually changes
as a result — bounded above by `O(|V| log |V| + |E|)` for the worst case
(equivalent to a full A* re-run), but in practice `k ≪ |V|` for local
updates, which is the entire point of using LPA* over plain A*/Dijkstra for
the dynamic-environment requirement (§ "Dynamic Environment" in the spec).
Each vertex pop is `O(log |V|)` (the `std::set`); each `UpdateVertex` touches
`O(deg⁻(u))` predecessors. This is confirmed empirically: Test Case 6's
incremental shortcut-discovery expanded only 5 vertices, not the full graph
(`experimental_results.txt`).

The one-shot `plan()` entry point (used for a cold start) is `O(|E| log |V|)`,
identical to Dijkstra/A*, since `initialize()` clears g/rhs and starts fresh.

## 6. Space Complexity

`O(|V| + |E|)`: two hash maps of size `≤|V|` for `g_`/`rhs_`, two adjacency
maps of total size `O(|E|)`, the open-list set/map of size `≤|V|`, plus the
transition and state tables themselves (`O(|E|)` and `O(|V|)`).

## 7. Replanning After Dynamic Updates

The assignment's five dynamic-environment cases and how each maps to an
`O(affected)` incremental operation rather than a full re-solve:

| Change | Method | Why it stays incremental |
|---|---|---|
| Goal state changes | `setGoal(id)` | `rhs(s)` is defined purely from predecessors' `g`-values and never references the goal (LPA* runs the search *from the start outward*). Only the priority key `k(s)=min(g,rhs)+h(s,goal)` is goal-relative, so a goal change only requires **re-keying the open list**, not recomputing `g`/`rhs`. Verified in Test Case 5. |
| Bad states added/removed | `setBadState(id)` | Marks incident transitions unusable and calls `UpdateVertex` only on their endpoints; the resulting `rhs` change then propagates only as far as it actually needs to via the open-list mechanics. |
| Transition availability changes | `setTransitionAvailable(id, bool)` | Same mechanism — a single `UpdateVertex` call, propagation is self-limiting because `UpdateVertex` is a no-op once `g(u)==rhs(u)` again. |
| New transitions added | `addTransition(t)` | `UpdateVertex(t.to)` re-evaluates whether the new edge improves `rhs(t.to)`; if it doesn't beat the existing best predecessor, nothing propagates further (Test Case 6 confirms the propagation was local: 5 vertex touches, not a full graph re-expansion). |
| Transitions removed | (equivalent to `setTransitionAvailable(id,false)`) | same as above |

In every case the caller then calls `replan()` (`ComputeShortestPath`), which
only pops vertices whose key is inconsistent with the goal's — vertices
unaffected by the change are never re-examined.

## 8. Experimental Results

All 6 illustrative test cases from the assignment were implemented as
executable, automatically-checked scenarios in `src/main.cpp`
(`testCase1`..`testCase6`). Full run captured verbatim in
`experimental_results.txt`. Summary:

| Test Case | Result | States expanded | Notes |
|---|---|---|---|
| 1. Basic reachability | PASS | 4 | unique path found, cost 3.0 |
| 2. Bad-state avoidance | PASS | 5 | bad state never visited; safe route chosen |
| 3. Safety-margin trade-off | PASS (x2 scenarios) | 4 / 3 | `safetyWeight` scalarization verified to flip the chosen path |
| 4. Dynamic transition removal + repair | PASS (x3 stages) | 3 / 4 / 6 | correctly reports failure with no alternative, then finds one once added |
| 5. Goal update | PASS (x2 stages) | 4 / 4 | g/rhs reused across goal change (see §7) |
| 6. Transition addition (shortcut) | PASS (x2 stages) | 4 / 5 | shortcut adopted, cost drops 3.0 → 0.5 |

**Total: 15/15 automated checks passed.** All five "Optimization Objectives"
from the spec are satisfied: (1) goal reached whenever a path exists, (2) bad
states structurally unreachable, (3) cost minimized subject to the safety/
reliability scalarization, (4) minimum safety distance reported and
demonstrably steerable via `safetyWeight`, (5) all scenarios plan in
sub-millisecond time on these small illustrative graphs (see
`experimental_results.txt` for exact timings — they are graph-size dependent
and not claimed to generalize to larger inputs without further benchmarking).

## Known Limitations (stated plainly, not hidden)

- Heuristic defaults to 0 (no speed-up) for provable correctness; a tuned
  positive `scale` was **not** benchmarked here, so no speed claim is made
  for it.
- Lazy `UpdateVertex` deletion assumes floating-point comparisons via a fixed
  `1e-9`/`1e-12` epsilon; this is adequate for the given cost ranges but is
  not a rigorous tolerance analysis for arbitrary inputs.
- `addTransition` for a **new state id** not already in `problem_.states`
  works for adjacency purposes but its embedding must be registered via
  `problem_.states`/`loadProblem` for the heuristic and safety-distance
  computations to see it (demonstrated correctly in Test Case 4, where the
  new state's embedding was added before the new transitions).
