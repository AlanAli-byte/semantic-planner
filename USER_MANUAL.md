# User Manual

## Build
Requires a C++17 compiler (tested: g++ 13.3, Ubuntu 24.04).

```bash
g++ -std=c++17 -O2 -Wall -Wextra -o planner_cli src/cli.cpp      # interactive tool (primary)
g++ -std=c++17 -O2 -Wall -Wextra -o planner_tests src/main.cpp   # automated regression suite
```

## Run — interactive mode (you supply the problem)
```bash
./planner_cli
```
Type commands at the `HELP`-listed grammar to build your own state space —
no test data is hardcoded. Example session:
```
STATE 0 0 0
STATE 1 1 0
STATE 2 2 0
TRANS 1 0 1 1.0
TRANS 2 1 2 1.0
INIT 0
GOAL 2
LOAD
PLAN
SETAVAIL 2 0
REPLAN
```
You can also pipe a script of commands non-interactively:
```bash
./planner_cli example_session.txt
```
`example_session.txt` (included) walks through a bad-state-avoidance scenario,
a dynamic edge failure, and a goal change, all typed as a user would.

Full command grammar is in `HELP` output (also duplicated at the top of
`src/cli.cpp`): `STATE`, `TRANS`, `INIT`, `GOAL`, `BAD`, `LOAD`, `PLAN`,
`REPLAN`, `SETBAD`, `SETAVAIL`, `SETCOST`, `ADDTRANS`, `SETGOAL`, `WEIGHTS`,
`SHOW`, `EXIT`. Malformed input (unknown id, out-of-range safety/reliability,
negative cost, missing INIT/GOAL before LOAD) is rejected with a specific
error and does not corrupt existing state.

## Run — automated regression suite (fixed test cases from the spec)
```bash
./planner_tests
```
Runs Test Cases 1–6 from the assignment automatically and prints
`[PASS]`/`[FAIL]` for every required condition, plus a final tally. Exit code
is `0` iff every check passed. Use this to verify the *engine* is correct;
use `planner_cli` to plan on your own problem instances.

## Using the library in your own code

```cpp
#include "include/lpastar_planner.hpp"

PlanningProblem p;
p.initialState = 0;
p.goalState    = 3;
p.badStates    = {2};
p.states       = { State(0,{0,0}), State(1,{1,0}), State(2,{2,0}), State(3,{3,0}) };
p.transitions  = { Transition(1,0,1, /*cost*/1.0, /*safety*/1.0, /*reliability*/1.0),
                    Transition(2,1,3, 1.0) };

LPAStarPlanner planner;
PlanningResult r = planner.plan(p);   // cold start
// r.success, r.statePath, r.transitionPath, r.totalCost, r.safetyScore

// --- incremental / dynamic environment usage ---
planner.loadProblem(p);                    // load once (does NOT reset g/rhs on reuse)
planner.setTransitionAvailable(2, false);   // edge goes down
planner.addTransition(Transition(9,0,3,0.4)); // new shortcut appears
planner.setGoal(2);                         // goal moves
PlanningResult r2 = planner.replan();       // O(affected), not from scratch
```

Tuning knobs: `planner.safetyWeight`, `planner.reliabilityWeight` (see
`DESIGN_REPORT.md` §4), `planner.setHeuristicScale(v)` (§3 — leave at default
0.0 unless you have proven a safe bound for your embeddings).

## Files
- `include/types.hpp` — `State`, `Transition`, `PlanningProblem`,
  `PlanningResult`, `Planner` (interface), `euclidean()`.
- `include/lpastar_planner.hpp` — `LPAStarPlanner : public Planner`, full
  LPA* implementation (`Initialize`, `UpdateVertex`, `ComputeShortestPath`)
  plus incremental mutators.
- `src/cli.cpp` — **interactive CLI**: user defines states/transitions/goal
  live and drives planning + dynamic updates via typed commands.
- `src/main.cpp` — Test Cases 1–6 + automated checks (`testCase1..6`, `main`),
  fixed data hardcoded on purpose — this is the regression suite, not the
  tool you plan your own problems with.
- `example_session.txt` — sample command script for `planner_cli`.
- `DESIGN_REPORT.md` — required design report (state representation, data
  structures, heuristic, safety computation, time/space complexity,
  replanning strategy, experimental results).
- `experimental_results.txt` — captured stdout of a full regression-suite run.
