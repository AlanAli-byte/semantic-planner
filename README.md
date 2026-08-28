# Safe Semantic Planner — LPA*

A browser-based interactive demonstration of a **Safe Semantic Planner in a finite Cartesian state space**, using **Lifelong Planning A\* (LPA\*)** for path planning and incremental replanning.

## 🌐 Live Demo

**Run the interactive planner directly in your browser:**

👉 **[Open the Live Demo](https://alanali-byte.github.io/semantic-planner/)**

No installation is required for the browser demonstration.

### Quick Start

For the fastest demonstration, open the **Live Demo** above. Select a scenario such as `TC1 · Basic Reachability`, click **Load scenario**, and then click **Plan (cold start)**. The interface also supports **Replan (incremental)** for dynamic changes.


> **Course:** PCCST503 – Machine Learning  
> **Assignment:** Assignment 1 — Design of a Safe Semantic Planner in a Finite Cartesian State Space  
> **Primary algorithm:** LPA\* (Lifelong Planning A\*)

---

## 1. Project Overview

The project models a finite state space as a directed graph:

- **States** are points in a Cartesian space.
- **Transitions** connect states and have:
  - cost
  - safety score
  - reliability score
  - availability
- One state is selected as the **initial state**.
- One state is selected as the **goal state**.
- Some states may be marked as **bad states** and are treated as hard constraints.

The planner searches for a path that:

1. reaches the goal when a valid path exists,
2. never enters a bad state,
3. minimizes transition cost subject to the selected safety/reliability weighting,
4. reports the minimum geometric distance from the selected path to bad states,
5. can update the solution incrementally when the environment changes.

The project includes an interactive SVG-based browser interface that allows the user to construct and modify the graph visually.

---

## 2. Main Features

### Interactive graph editor

The interface supports:

- adding states,
- moving states,
- adding directed transitions,
- editing transition attributes,
- deleting states/transitions,
- setting the initial state,
- setting the goal state,
- marking/unmarking bad states.

### Planning controls

Two planning modes are provided:

- **Plan (cold start)** — clears the previous search state and solves the current problem from scratch.
- **Replan (incremental)** — reuses the existing LPA\* `g`/`rhs` state and updates only the part of the search affected by changes.

### Objective controls

The interface provides:

- **Safety weight**
- **Reliability weight**
- **Animation speed**

Increasing the safety weight makes low-safety transitions more expensive. Increasing the reliability weight makes unreliable transitions more expensive.

### Visual explanation

The graph visually distinguishes:

- initial state,
- goal state,
- bad states,
- expanded states,
- selected path,
- unavailable transitions.

The result panel reports:

- total path cost,
- safety distance,
- number of expanded states,
- planning time.

---

## 3. Problem Formulation

Let


$$
S = \{s_1,s_2,\ldots,s_n\} \subset \mathbb{R}^d
$$

be the finite set of states.

Each state has an embedding:


$$
s_i=(x_1,x_2,\ldots,x_d)
$$

The planner receives:

- initial state $s_I$,
- goal state $s_G$,
- bad-state set $B$,
- directed transitions $T$.

Each transition contains:


$$
t=(from,to,cost,safety,reliability,available)
$$

The assignment requires the planner to reach the goal while avoiding bad states, while optimizing cost and safety-related objectives.

---

## 4. Why LPA*?

A conventional Dijkstra/A\* search is effective when the graph does not change.

This assignment specifically includes a **dynamic environment**, where:

- the goal can change,
- bad states can change,
- transition availability can change,
- new transitions can be inserted,
- transitions can be removed.

Re-running the entire search after every small change wastes previous search information.

**LPA\*** solves this by maintaining two values for each state:

- `g(s)` — the current best-known path cost,
- `rhs(s)` — one-step lookahead value.

A state is locally consistent when:


$$
g(s)=rhs(s)
$$

When the graph changes, only affected vertices become inconsistent and are placed in the priority queue. The algorithm then propagates the consequences of the change.

This makes LPA\* particularly suitable for incremental replanning.

---

## 5. Effective Transition Weight

The implementation combines transition cost, reliability, and safety into one minimization weight:


$$
w(u,v)
=
cost(u,v)
\left[1+W_R(1-reliability(u,v))\right]
+
W_S(1-safety(u,v))
$$

where:

- $W_R$ = reliability weight,
- $W_S$ = safety weight.

Therefore:

### High reliability

If reliability = 1:


$$
1+W_R(1-1)=1
$$

so no reliability penalty is added.

### Low reliability

If reliability < 1, the transition receives an additional cost.

### High safety

If safety = 1:


$$
W_S(1-1)=0
$$

so no safety penalty is added.

### Low safety

If safety < 1, the transition receives a larger penalty as $W_S$ increases.

This converts the multi-objective problem into a scalar edge-weight minimization problem.

---

## 6. Hard Safety vs Soft Safety

These are deliberately treated as **two different concepts**.

### Hard safety constraint

A transition is unusable if:

- it is unavailable,
- its source is a bad state,
- its destination is a bad state.

Therefore a bad state cannot be selected simply because it is cheap.

Conceptually:

```text
usable(t) =
    available(t)
    AND source is not bad
    AND destination is not bad
```

This guarantees that the selected plan never intentionally routes through a bad state.

### Soft safety objective

Among paths that satisfy the hard constraint, transition-level safety values influence the effective cost.

The final result also reports the minimum Euclidean distance between any visited state and the nearest bad state.

These two measurements should not be confused:

- **hard constraint:** "Can this state/transition be used?"
- **soft safety:** "How strongly should the planner prefer safer transitions?"

---

## 7. Heuristic

The general design supports:


$$
h(s)=
\operatorname{EuclideanDistance}(embedding(s),embedding(goal))
\times scale
$$

The conservative default is:


$$
scale=0
$$

so:


$$
h(s)=0
$$

This makes the implementation Dijkstra-equivalent while remaining admissible and consistent without assuming an unsafe relationship between geometric distance and the effective edge weights.

A positive heuristic scale can be used only when a valid lower bound on effective edge cost per unit distance is established.

**Why this conservative choice matters:** an arbitrary positive heuristic could overestimate the remaining cost and destroy the optimality guarantee.

---

## 8. LPA* Data Structures

The design uses:

| Structure | Purpose |
|---|---|
| `stateById_` | State ID → state |
| `transitionsById_` | Transition ID → transition |
| `successors_` | Outgoing adjacency |
| `predecessors_` | Incoming adjacency used for `rhs` |
| `g_` | Current path-cost estimates |
| `rhs_` | One-step lookahead values |
| `open_` | Priority structure containing inconsistent states |
| `keyOfOpen_` | Tracks the key associated with an open state |

The average hash-map lookup is $O(1)$. Priority-queue insertion/removal is $O(\log |V|)$.

---

## 9. LPA* Search Logic

The search starts by setting:


$$
rhs(start)=0
$$

and inserting the start state into the open structure.

For a non-start state:


$$
rhs(s)=
\min_{(u,s)}
\left[g(u)+w(u,s)\right]
$$

over usable incoming transitions.

The priority key is based on:


$$
k(s)=
\left[
\min(g(s),rhs(s))+h(s),
\min(g(s),rhs(s))
\right]
$$

The algorithm repeatedly processes the state with the smallest key.

If:


$$
g(s)>rhs(s)
$$

the state becomes more consistent by setting:


$$
g(s)=rhs(s)
$$

If:


$$
g(s)\le rhs(s)
$$

the state is made locally inconsistent again and its successors are updated.

The process stops when the goal is consistent and no open state has a smaller priority key.

---

## 10. Incremental Replanning

The important advantage of LPA\* appears after a change.

### Goal changes

The existing `g` and `rhs` values can be reused. The goal affects the priority key through the heuristic/termination condition, so the open-list keys are recalculated rather than rebuilding the complete search.

### Bad state changes

When a state becomes bad or safe, incident transitions change usability. Only affected vertices are updated first; the inconsistency then propagates as required.

### Transition availability changes

Changing a transition from available to unavailable, or vice versa, triggers an update of the affected destination and any necessary propagation.

### New transition

A new edge can improve the destination's `rhs` value. If it improves the current solution, the improvement propagates through the existing LPA\* search state.

### Removed transition

Removing an edge is equivalent to making that transition unavailable, after which affected vertices are updated.

This is the main reason LPA\* was selected instead of repeatedly running a cold-start search.

---

# 11. How to Use the Web Interface

## Step 1 — Choose a scenario

Use the **Scenario** dropdown.

Available scenarios:

1. `TC1 · Basic Reachability`
2. `TC2 · Hard Bad-State Avoidance`
3. `TC3 · Safety-Weight Scalarisation`
4. `TC4 · Dynamic Edge Loss & Recovery`
5. `TC5 · Incremental Goal Change`
6. `TC6 · Shortcut Discovery`
7. `Blank — build from scratch`

Click **Load scenario**.

---

## Step 2 — Select an editing mode

### `+ State`

Click an empty area of the canvas to create a new state.

### `+ Transition`

Drag from one state to another.

A transition editor appears where you can set:

- cost,
- safety,
- reliability.

Click **Create**.

### `Move`

Drag a state to a new position.

### `Set init`

Click a state to make it the initial state.

### `Set goal`

Click a state to make it the goal.

### `Toggle bad`

Click a state to mark it bad or safe.

### `Delete`

Click a state or transition to delete it.

---

## Step 3 — Adjust objective weights

### Safety weight

Range:


$$
0 \le W_S \le 6
$$

Use this to control how strongly low-safety transitions are penalized.

### Reliability weight

Range:


$$
0 \le W_R \le 6
$$

Use this to control the effect of transition reliability.

### Animation speed

Controls only the visualization speed; it does not change the planner's objective.

---

## Step 4 — Run the planner

### Plan (cold start)

Use this when:

- starting a new graph,
- changing the initial state,
- wanting a complete fresh search.

It clears the existing LPA\* search state.

### Replan (incremental)

Use this after dynamic changes such as:

- changing the goal,
- changing a bad state,
- changing transition availability,
- adding a transition,
- changing transition attributes.

It attempts to reuse the existing `g`/`rhs` information.

---

# 12. Understanding the Display

### State colours/labels

- **INIT** — initial state
- **GOAL** — goal state
- **BAD** — hard-avoid state
- **Expanded** — state processed by the search
- **Selected safe path** — final path

### Transition labels

A transition displays:

```text
c<cost> s<safety> r<reliability>
```

For example:

```text
c2 s0.8 r0.9
```

means:

- cost = 2
- safety = 0.8
- reliability = 0.9

An unavailable transition is shown as disabled/dashed.

---

# 13. Built-in Test Scenarios

## TC1 — Basic Reachability

Expected graph:

```text
S → A → B → G
```

There is a unique valid path.

Expected result:

```text
S → A → B → G
cost = 3
```

---

## TC2 — Hard Bad-State Avoidance

Two paths exist:

```text
S → A → X → G
```

where `X` is bad, and:

```text
S → C → D → G
```

The first path is structurally forbidden.

The planner must select the second path.

---

## TC3 — Safety-Weight Scalarisation

Two valid paths exist:

- one is cheaper but close to a bad state,
- one is more expensive but farther from the bad state.

With:

```text
safetyWeight = 0
```

the cost-dominant path is selected.

With:

```text
safetyWeight = 5
```

the safer path becomes preferable.

---

## TC4 — Dynamic Edge Loss and Recovery

Initially:

```text
S → A → G
```

Then:

```text
A → G
```

becomes unavailable.

The planner first reports that no path exists if no alternative is available.

After adding:

```text
S → B → G
```

the planner finds the alternative route.

This demonstrates incremental repair.

---

## TC5 — Incremental Goal Change

The initial goal is changed while the planner already has search information.

The revised goal is reached without rebuilding all `g`/`rhs` information.

---

## TC6 — Shortcut Discovery

Baseline:

```text
S → A → B → G
cost = 3
```

A shortcut is then added:

```text
S → G
cost = 0.5
```

The incremental planner discovers the new lower-cost solution.

---

# 14. Experimental Results

The automated verification suite contains all six assignment test cases.

| Test Case | Result | Expanded | Key result |
|---|---:|---:|---|
| TC1 Basic Reachability | PASS | 4 | Path found, cost 3.0 |
| TC2 Bad-State Avoidance | PASS | 5 | Bad state avoided |
| TC3 Safety Trade-off | PASS | 4 / 3 | Weight changes selected path |
| TC4 Dynamic Transition | PASS | 3 / 4 / 6 | Failure and recovery handled |
| TC5 Goal Update | PASS | 4 / 4 | New goal reached incrementally |
| TC6 Shortcut Addition | PASS | 4 / 5 | Cost reduced 3.0 → 0.5 |

### Exact measured results

**TC1**

```text
path = [0 → 1 → 2 → 3]
cost = 3
safety distance = 0
expanded = 4
time = 0.0031 ms
```

**TC2**

```text
path = [0 → 3 → 5 → 4]
cost = 3
safety distance = 1.4142
expanded = 5
time = 0.0027 ms
```

**TC3**

Cost-dominant:

```text
safetyWeight = 0
path = [0 → 1 → 3]
cost = 2
safety distance = 0.1
expanded = 4
time = 0.0022 ms
```

Safety-dominant:

```text
safetyWeight = 5
path = [0 → 2 → 3]
cost = 4
safety distance = 1.0
expanded = 3
time = 0.0016 ms
```

**TC4**

Initial:

```text
success = true
path = [0 → 1 → 2]
cost = 2
expanded = 3
```

After edge loss:

```text
success = false
path = []
cost = infinity
expanded = 4
```

After alternative transition is added:

```text
success = true
path = [0 → 3 → 2]
cost = 3
expanded = 6
```

**TC5**

Original goal:

```text
path = [0 → 1 → 2]
cost = 2
```

New goal:

```text
path = [0 → 3]
cost = 1
```

**TC6**

Before shortcut:

```text
path = [0 → 1 → 2 → 3]
cost = 3
```

After shortcut:

```text
path = [0 → 3]
cost = 0.5
```

### Overall verification

```text
15 passed
0 failed
ALL CONDITIONS SATISFIED
```

The timings are for the small illustrative graphs used by the assignment and should **not** be interpreted as a general benchmark for large graphs.

---

# 15. Complexity

Let:

- $V$ = number of states,
- $E$ = number of transitions.

### Space


$$
O(|V|+|E|)
$$

because the planner stores state tables, transitions, adjacency information, `g`, `rhs`, and the open structure.

### Cold-start planning

The implementation is Dijkstra-equivalent with the default zero heuristic, giving a worst-case bound comparable to:


$$
O(|E|\log|V|)
$$

for the graph sizes and priority structure used here.

### Incremental replanning

After a local change, the work depends primarily on the number of vertices whose values/keys are actually affected.

If $k$ vertices require processing, the local search work is approximately:


$$
O(k\log k)
$$

with a worst case that can approach a full graph re-solve.

The important practical property is:


$$
k \ll |V|
$$

for genuinely local changes.

---

# 16. Failure / Conditional Cases

The planner does not assume that a path always exists.

### No initial state

Planning cannot start until an initial state is selected.

### No goal

Planning cannot terminate meaningfully without a goal.

### Unreachable goal

If every usable route to the goal is blocked, the planner reports failure rather than inventing a path.

### Bad state on a route

Any transition touching a bad state is unusable.

### Unavailable transition

An unavailable transition is excluded from search.

### Dynamic edge loss

If the current route becomes unavailable and no alternative exists, the planner correctly reports no path. If an alternative is subsequently added, replanning can recover a solution.

### Initial-state change

Changing the initial state requires a cold-start planning state because the LPA\* `rhs` initialization is defined relative to the initial state.

### Empty/invalid graph

A useful plan requires valid states, a selected initial state, a selected goal, and usable transitions connecting them.

---

# 17. Known Limitations

1. **Default heuristic is zero.**  
   This prioritizes correctness over heuristic speed-up.

2. **Positive heuristic scaling requires a proven bound.**  
   An arbitrary non-zero scale could make the heuristic inconsistent or inadmissible.

3. **Safety is represented in two ways.**  
   The hard constraint uses bad-state membership, while the optimization uses transition-level safety values. These are related but not identical measurements.

4. **Safety score is an evaluation metric.**  
   The reported minimum Euclidean distance is calculated after path extraction; it is not directly the quantity minimized by the LPA\* search.

5. **Timing results are graph-size dependent.**  
   The sub-millisecond values in the verification suite are not a benchmark for large-scale planning.

6. **Floating-point comparisons use small epsilons.**  
   The implementation uses fixed tolerances for equality/key comparisons.

7. **State embeddings must exist for geometric calculations.**  
   A newly referenced state must be registered correctly for heuristic and safety-distance calculations.

---

# 18. Assignment Deliverables

The assignment specifies five deliverables:

1. C++ source code
2. Design report
3. Experimental results
4. User manual
5. Demonstration

This README combines the **project explanation and user manual**.

Recommended repository structure:

```text
semantic-planner/
├── README.md
├── DESIGN_REPORT.md
├── experimental_results.txt
├── index.html
├── include/
│   ├── types.hpp
│   └── lpastar_planner.hpp
└── src/
    └── main.cpp
```

**Important:** if the submitted GitHub repository is intended to satisfy the C++ source-code deliverable, make sure the actual `include/` and `src/` files are committed. The browser `index.html` demonstrates the planner, but by itself it is not a substitute for the assignment's requested C++ source-code deliverable.

---

# 19. Demonstration Procedure

For a short assignment demonstration:

### Demonstration A — Basic planning

1. Load `TC1`.
2. Click **Plan (cold start)**.
3. Show the selected path.
4. Point out total cost, expanded states, and planning time.

### Demonstration B — Hard safety

1. Load `TC2`.
2. Run the planner.
3. Show that the path does not enter the bad state.

### Demonstration C — Multi-objective trade-off

1. Load `TC3`.
2. Set safety weight to `0`.
3. Plan and record the cheaper/closer path.
4. Set safety weight to `5`.
5. Replan.
6. Show that the selected path changes to the farther/safer route.

### Demonstration D — Incremental replanning

1. Load `TC4`.
2. Run the initial plan.
3. Disable the critical transition.
4. Replan.
5. Show failure when no alternative exists.
6. Add the alternative transition.
7. Replan.
8. Show recovery.

### Demonstration E — Shortcut discovery

1. Load `TC6`.
2. Run the baseline plan.
3. Add the shortcut.
4. Use incremental replanning.
5. Show cost reduction from `3.0` to `0.5`.

---

# 20. Conclusion

The project implements the main requirements of the Safe Semantic Planner assignment using LPA\*. The key design choice is to treat **bad states as hard constraints** while using **cost, safety, and reliability as soft optimization factors**. LPA\* maintains reusable search information so that changes to the goal or graph can be handled incrementally instead of always starting from zero.

The verification suite reports:

**15/15 automated checks passed, 0 failures.**

---

---

## Student Details

**Name:** Alan P Ali  
**Roll No.:** TCR24CS008  
**Class:** CSE/S3

