# Design Report — Safe Semantic Planner using LPA*

## PCCST503 – Machine Learning
### Assignment 1 — Design of a Safe Semantic Planner in a Finite Cartesian State Space

---

## 1. Introduction

The assignment requires a generic planning algorithm that computes a safe path through a finite set of states embedded in a Cartesian space. The planner must reach a specified goal, avoid bad states, minimize transition cost, consider safety and reliability, and support efficient replanning when the environment changes.

This project uses **Lifelong Planning A\* (LPA\*)**. LPA\* was selected because the assignment explicitly includes dynamic changes to the goal, bad states, transition availability, and graph topology.

The implementation has two related parts:

1. a planner implementation following the assignment's `State`, `Transition`, `PlanningProblem`, `PlanningResult`, and `Planner` abstractions;
2. an interactive browser visualization that exposes the same planning concepts through an editable finite state graph.

---

# 2. Problem Definition

Let

\[
S=\{s_1,s_2,\ldots,s_n\}\subset\mathbb{R}^d
\]

be the finite state space.

Each state is represented by a vector:

\[
s_i=(x_1,x_2,\ldots,x_d)
\]

The planner receives:

- initial state \(s_I\),
- goal state \(s_G\),
- bad-state set \(B\),
- directed transition set \(T\).

Each transition has:

- cost,
- safety score,
- reliability score,
- availability flag.

The required optimization objectives are:

1. reach the goal whenever a valid path exists;
2. never visit a bad state;
3. minimize total transition cost;
4. maximize the minimum Euclidean distance from visited states to the nearest bad state;
5. produce the result within reasonable execution time.

The assignment gives the example objective:

\[
Score(P)=\alpha G-\beta C+\gamma D+\delta R
\]

where \(G\) represents goal completion, \(C\) cumulative cost, \(D\) minimum safety distance, and \(R\) cumulative reliability.

---

# 3. State Representation

A state is represented by an identifier and a Cartesian embedding.

Conceptually:

```cpp
State {
    uint64_t id;
    std::vector<double> embedding;
};
```

The identifier is used for graph and search-table lookup. The embedding provides the geometric representation needed for Euclidean-distance calculations.

Bad states are not represented as a separate state class. Instead, ordinary state IDs are placed in the bad-state set.

This has two advantages:

1. the state-space definition remains unchanged;
2. a state can be marked or unmarked bad without changing the underlying graph structure.

The browser implementation similarly stores states using:

```text
id
x
y
bad
```

because the visualization uses a two-dimensional Cartesian canvas.

---

# 4. Transition Representation

A transition contains:

```text
id
from
to
cost
safety
reliability
available
```

The transition is directed, so \(u\rightarrow v\) does not automatically imply \(v\rightarrow u\).

The availability flag represents dynamic environmental conditions. A transition may exist in the graph but temporarily be unavailable for planning.

---

# 5. Data Structures

The implementation uses hash-based structures for state, transition, adjacency, and LPA\* values.

| Structure | Purpose | Average / stated complexity |
|---|---|---|
| `stateById_` | ID → State | \(O(1)\) average lookup |
| `transitionsById_` | ID → Transition | \(O(1)\) average lookup |
| `successors_` | outgoing transitions | \(O(deg)\) traversal |
| `predecessors_` | incoming transitions | \(O(deg)\) traversal |
| `g_` | current path estimate | \(O(1)\) average lookup |
| `rhs_` | one-step lookahead | \(O(1)\) average lookup |
| `open_` | ordered LPA\* queue | \(O(\log |V|)\) insert/erase |
| `keyOfOpen_` | open-state/key tracking | \(O(1)\) average lookup |

The ordered open structure is important because LPA\* must repeatedly obtain the state with the smallest key while also being able to remove and reinsert states when their keys change.

Overall storage is:

\[
O(|V|+|E|)
\]

---

# 6. Why LPA*?

A cold-start shortest-path algorithm discards previous search information after every environmental change.

For example, suppose:

```text
S → A → G
```

is initially optimal.

If `(A,G)` becomes unavailable, a conventional approach can simply run a new search from `S`. However, much of the previous computation may still be useful.

LPA\* maintains persistent search information through:

- \(g(s)\)
- \(rhs(s)\)
- an ordered open structure.

A vertex is locally consistent when:

\[
g(s)=rhs(s)
\]

After a local graph change, only affected vertices become inconsistent. LPA\* processes those inconsistencies and propagates their consequences.

Therefore LPA\* is well matched to the assignment's dynamic-environment requirement.

---

# 7. LPA* Equations

For the initial state:

\[
rhs(s_I)=0
\]

For other states:

\[
rhs(s)=
\min_{(u,s)\in T}
\left[g(u)+w(u,s)\right]
\]

where only usable transitions are considered.

The priority key is:

\[
k(s)=
\left[
\min(g(s),rhs(s))+h(s),
\min(g(s),rhs(s))
\right]
\]

The search repeatedly selects the smallest key.

If:

\[
g(u)>rhs(u)
\]

then:

\[
g(u)\leftarrow rhs(u)
\]

and successors are updated.

Otherwise, the state is made inconsistent by setting:

\[
g(u)\leftarrow\infty
\]

and its affected neighbors are updated.

The search terminates when the goal is locally consistent and no remaining open state has a key that requires further processing.

---

# 8. Heuristic Function

The general heuristic is:

\[
h(s)=
\operatorname{EuclideanDistance}
(embedding(s),embedding(goal))
\times scale
\]

The implementation uses:

\[
scale=0
\]

as the conservative default.

Therefore:

\[
h(s)=0
\]

and the search is Dijkstra-equivalent.

This is intentional. A positive geometric heuristic is safe only if its scale is no larger than a valid lower bound on effective edge cost per unit Euclidean distance.

For arbitrary input embeddings and arbitrary safety/reliability weighting, that bound has not been established. Using an unjustified positive heuristic could make the heuristic inadmissible or inconsistent and could therefore compromise optimality.

The zero heuristic sacrifices heuristic acceleration in exchange for a conservative correctness guarantee.

---

# 9. Safety Computation

The project distinguishes **hard safety** from **soft safety**.

## 9.1 Hard safety constraint

A transition is usable only if:

\[
available(t)=true
\]

and neither endpoint is a bad state.

Conceptually:

\[
usable(t)=
available(t)
\land
\neg bad(from(t))
\land
\neg bad(to(t))
\]

This is a structural constraint rather than a numerical penalty.

Consequently, increasing or decreasing safety/reliability weights cannot make a bad state become legal.

---

## 9.2 Soft safety and reliability

The effective edge weight is:

\[
w(u,v)=
cost(u,v)
\left[1+W_R(1-reliability(u,v))\right]
+
W_S(1-safety(u,v))
\]

where:

- \(W_R\) is the reliability weight;
- \(W_S\) is the safety weight.

When reliability is 1, its penalty is zero.

When safety is 1, its safety penalty is zero.

A lower safety score therefore increases the effective cost when \(W_S>0\).

This allows the planner to select a more expensive but safer route when the safety weight is sufficiently high.

---

# 10. Safety-Distance Evaluation

After a path is extracted, the implementation calculates the minimum Euclidean distance between every visited state and every bad state.

The reported value is:

\[
D(P)=
\min_{s\in P,\;b\in B}
\|embedding(s)-embedding(b)\|_2
\]

The browser result converts this distance into a displayed `safetyScore` using the implementation's normalization.

This metric is an **evaluation metric**, not the exact quantity directly minimized by LPA\*. The search instead uses the transition-level `safety` attribute as a proxy in the effective edge weight.

This distinction is important because transition safety and geometric distance are not mathematically identical.

---

# 11. Path Extraction

After the goal becomes reachable, the implementation reconstructs a path backward from the goal.

For the current state \(v\), it searches usable predecessors \(u\) satisfying the minimum:

\[
g(u)+w(u,v)
\]

The selected predecessor becomes the next state in the reverse reconstruction.

The resulting state sequence is reversed to obtain:

\[
s_I\rightarrow\cdots\rightarrow s_G
\]

The planner reports:

- success/failure,
- state path,
- transition path,
- total transition cost,
- safety score.

---

# 12. Dynamic Replanning

The assignment explicitly requires the ability to handle changing conditions.

## 12.1 Goal change

The goal can be changed after a plan exists.

The existing \(g\) and \(rhs\) information is retained. The goal-relative priority information is updated rather than reconstructing all search values from zero.

This was verified experimentally in Test Case 5.

---

## 12.2 Bad-state changes

When a state changes between safe and bad, the usability of incident transitions changes.

Affected vertices are updated through `UpdateVertex`, and LPA\* propagates the resulting inconsistency only as far as necessary.

---

## 12.3 Transition availability

When a transition becomes unavailable or available again, the affected destination's `rhs` value is reconsidered.

The update then propagates through inconsistent vertices.

---

## 12.4 New transition

When a new transition is inserted, its destination is evaluated again.

If:

\[
g(from)+w(from,to)
\]

is better than the existing `rhs(to)`, the new transition improves the solution and the improvement can propagate.

If it does not improve the current solution, unnecessary propagation is avoided.

---

## 12.5 Removed transition

Removing a transition is equivalent to making it unavailable from the planner's perspective. The affected state is updated and inconsistencies propagate if required.

---

# 13. Browser Demonstration Interface

The browser application exposes the planner through an interactive SVG canvas.

The available scenario presets are:

- TC1 Basic Reachability
- TC2 Hard Bad-State Avoidance
- TC3 Safety-Weight Scalarisation
- TC4 Dynamic Edge Loss & Recovery
- TC5 Incremental Goal Change
- TC6 Shortcut Discovery
- Blank — build from scratch

Editing operations include:

- add state,
- add transition,
- move state,
- set initial state,
- set goal,
- toggle bad state,
- delete.

The interface also provides sliders for:

- safety weight,
- reliability weight,
- animation speed.

The user can select either cold-start planning or incremental replanning.

---

# 14. Illustrative Test Cases

## Test Case 1 — Basic Reachability

Expected:

```text
S → A → B → G
```

The planner should find the unique valid path.

Observed:

```text
path = [0 → 1 → 2 → 3]
cost = 3
expanded = 4
time = 0.0031 ms
```

Result: **PASS**.

---

## Test Case 2 — Bad-State Avoidance

Two alternatives are provided:

```text
S → A → X → G
```

where \(X\) is bad, and:

```text
S → C → D → G
```

Observed:

```text
path = [0 → 3 → 5 → 4]
cost = 3
safety distance = 1.4142
expanded = 5
time = 0.0027 ms
```

The bad state was not visited.

Result: **PASS**.

---

## Test Case 3 — Safety Margin Trade-off

### Cost-dominant configuration

\[
W_S=0
\]

Observed:

```text
path = [0 → 1 → 3]
cost = 2
safety distance = 0.1
expanded = 4
time = 0.0022 ms
```

The cheaper route is selected.

### Safety-dominant configuration

\[
W_S=5
\]

Observed:

```text
path = [0 → 2 → 3]
cost = 4
safety distance = 1.0
expanded = 3
time = 0.0016 ms
```

The farther/safer route is selected.

Result: **PASS**.

This demonstrates that safety weighting changes the optimization trade-off.

---

## Test Case 4 — Dynamic Transition

Initial condition:

```text
S → A → G
```

Observed:

```text
path = [0 → 1 → 2]
cost = 2
expanded = 3
```

Then `(A,G)` is made unavailable.

Observed:

```text
success = false
path = []
cost = infinity
expanded = 4
```

This is the correct result because no alternative exists yet.

After adding:

```text
S → B → G
```

the planner produces:

```text
path = [0 → 3 → 2]
cost = 3
expanded = 6
```

Result: **PASS**.

---

## Test Case 5 — Goal Update

Initial goal:

```text
path = [0 → 1 → 2]
cost = 2
```

After changing the goal:

```text
path = [0 → 3]
cost = 1
```

The existing LPA\* search state is reused rather than completely reinitialized.

Result: **PASS**.

---

## Test Case 6 — Transition Addition

Baseline:

```text
S → A → B → G
cost = 3
```

A shortcut is added:

```text
S → G
cost = 0.5
```

The incremental planner adopts:

```text
S → G
cost = 0.5
```

Result: **PASS**.

---

# 15. Experimental Summary

| Test | Result | Expanded states | Main verification |
|---|---|---:|---|
| TC1 | PASS | 4 | Goal reached |
| TC2 | PASS | 5 | Bad state avoided |
| TC3 | PASS | 4 / 3 | Safety weighting changes route |
| TC4 | PASS | 3 / 4 / 6 | Dynamic failure and recovery |
| TC5 | PASS | 4 / 4 | Goal changed incrementally |
| TC6 | PASS | 4 / 5 | Shortcut discovered |

The complete verification run produced:

\[
15/15\text{ checks passed}
\]

with:

\[
0\text{ failures}
\]

The measured test-case planning times were all below 0.004 ms on the small illustrative graphs.

These timings are implementation/environment/graph-size dependent and are not claimed to represent performance on large planning problems.

---

# 16. Complexity Analysis

Let:

- \(V=|S|\),
- \(E=|T|\).

## Space

The state table, transition table, adjacency structures, `g`, `rhs`, and open structure require:

\[
O(V+E)
\]

space.

## Cold start

With a zero heuristic, the search is Dijkstra-equivalent. With the ordered set priority structure, the one-shot implementation has a worst-case graph-search cost comparable to:

\[
O(E\log V)
\]

for nonnegative effective edge weights.

## Incremental update

For a local change, let \(k\) denote the number of vertices whose search keys/values actually need processing.

The practical incremental work is approximately:

\[
O(k\log k)
\]

with a worst case approaching a complete re-solve:

\[
O(V\log V+E)
\]

The key advantage is that local changes frequently have:

\[
k\ll V
\]

so previous computation is reused.

---

# 17. Conditional and Failure Cases

The planner explicitly handles cases in which a valid solution does not exist.

### No initial state

No search can be initialized.

### No goal

A goal-directed result cannot be produced.

### Goal unreachable

The planner returns failure rather than generating an invalid path.

### Bad-state route

Any transition touching a bad state is excluded.

### Unavailable transition

The transition is excluded.

### Dynamic route failure

If the current route is disabled and no alternative exists, the planner reports failure.

### Recovery

If a new valid transition is later added, incremental replanning can discover a new route.

### Initial-state modification

Changing the initial state requires a cold-start search because the LPA\* initialization is defined relative to the start.

---

# 18. Verification of Assignment Objectives

The assignment objectives are addressed as follows:

| Assignment objective | Implementation |
|---|---|
| Reach goal | LPA\* shortest-path computation |
| Never visit bad state | Structural `edgeUsable()` constraint |
| Minimize cost | Effective edge-weight minimization |
| Maximize safety margin | Safety-weight scalarization plus reported minimum geometric distance |
| Reasonable execution time | Incremental LPA\* and small verified graphs |

The six supplied illustrative scenarios were implemented and verified automatically.

---

# 19. Limitations

### 19.1 Zero default heuristic

The default heuristic does not accelerate search because:

\[
h(s)=0
\]

This is a deliberate conservative choice.

### 19.2 Positive heuristic requires calibration

A positive heuristic scale requires a proven lower bound on effective cost per unit distance.

### 19.3 Safety proxy

The search optimizes transition-level safety values, while the final geometric safety distance is calculated separately.

### 19.4 Floating-point tolerance

The implementation uses small fixed epsilons for floating-point equality and key comparisons. This is appropriate for the demonstrated value ranges but is not a complete numerical-stability analysis for arbitrary input ranges.

### 19.5 Benchmark scale

The experimental graphs are intentionally small. The measured sub-millisecond times should not be generalized to large state spaces.

---

# 20. Conclusion

The project implements a safe path planner for a finite Cartesian state space using LPA\*.

The central design decisions are:

1. represent states as Cartesian embeddings with stable IDs;
2. represent transitions with cost, safety, reliability, and availability;
3. treat bad states as a hard constraint;
4. combine cost, safety, and reliability through an effective edge weight;
5. use a conservative zero heuristic for correctness;
6. retain `g` and `rhs` values across supported dynamic updates;
7. validate the implementation through the six assignment scenarios.

The experimental verification produced:

**15/15 checks passed and 0 failed.**

The project therefore demonstrates the required graph-search, safety, optimization, software-engineering, and experimental-evaluation aspects of the assignment.
