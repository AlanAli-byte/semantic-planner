#include "../include/lpastar_planner.hpp"
#include <iostream>
#include <iomanip>
#include <cassert>
#include <sstream>

static int g_pass = 0, g_fail = 0;

void check(bool cond, const std::string& label) {
    std::cout << (cond ? "  [PASS] " : "  [FAIL] ") << label << "\n";
    if (cond) g_pass++; else g_fail++;
}

std::string pathStr(const std::vector<uint64_t>& p) {
    std::ostringstream os;
    for (size_t i = 0; i < p.size(); ++i) { os << p[i]; if (i + 1 < p.size()) os << " -> "; }
    return os.str();
}

void printResult(const PlanningResult& r) {
    std::cout << "  success=" << (r.success ? "true" : "false")
               << " path=[" << pathStr(r.statePath) << "]"
               << " cost=" << r.totalCost
               << " safetyDist=" << r.safetyScore
               << " expanded=" << r.statesExpanded
               << " time=" << std::fixed << std::setprecision(4) << r.planningTimeMs << "ms\n";
}

// ---------------------------------------------------------------------------
// Test Case 1: Basic reachability. S(0) -> A(1) -> B(2) -> G(3), unique path.
// ---------------------------------------------------------------------------
void testCase1() {
    std::cout << "\n=== Test Case 1: Basic Reachability ===\n";
    PlanningProblem p;
    p.initialState = 0; p.goalState = 3;
    p.states = { {0,{0,0}}, {1,{1,0}}, {2,{2,0}}, {3,{3,0}} };
    p.transitions = {
        {1,0,1,1.0}, {2,1,2,1.0}, {3,2,3,1.0}
    };
    LPAStarPlanner planner;
    auto r = planner.plan(p);
    printResult(r);
    check(r.success, "planner reaches goal");
    check(r.statePath == std::vector<uint64_t>({0,1,2,3}), "unique path S->A->B->G found");
    check(std::fabs(r.totalCost - 3.0) < 1e-9, "total cost == 3.0");
}

// ---------------------------------------------------------------------------
// Test Case 2: Bad state avoidance.
// S(0)->A(1)->X(2, BAD)->G(4)   vs   S(0)->C(3)->D(5)->G(4)
// ---------------------------------------------------------------------------
void testCase2() {
    std::cout << "\n=== Test Case 2: Bad State Avoidance ===\n";
    PlanningProblem p;
    p.initialState = 0; p.goalState = 4;
    p.badStates = {2};
    p.states = { {0,{0,0}}, {1,{1,1}}, {2,{2,1}}, {3,{1,-1}}, {5,{2,-1}}, {4,{3,0}} };
    p.transitions = {
        {1,0,1,1.0}, {2,1,2,1.0}, {3,2,4,1.0},   // via bad state X
        {4,0,3,1.0}, {5,3,5,1.0}, {6,5,4,1.0}    // safe alternative
    };
    LPAStarPlanner planner;
    auto r = planner.plan(p);
    printResult(r);
    check(r.success, "planner reaches goal");
    bool visitsBad = false;
    for (auto s : r.statePath) if (s == 2) visitsBad = true;
    check(!visitsBad, "bad state X never visited");
    check(r.statePath == std::vector<uint64_t>({0,3,5,4}), "safe alternative path selected");
}

// ---------------------------------------------------------------------------
// Test Case 3: Safety margin trade-off.
// Path 1: lower cost, close to a bad state (low safety attribute).
// Path 2: higher cost, far from bad states (high safety attribute).
// ---------------------------------------------------------------------------
void testCase3() {
    std::cout << "\n=== Test Case 3: Safety Margin Trade-off ===\n";
    PlanningProblem p;
    p.initialState = 0; p.goalState = 3;
    p.badStates = {4};
    p.states = { {0,{0,0}}, {1,{1,0.1}}, {2,{1,5}}, {3,{2,0}}, {4,{1,0}} };
    // Path A: 0->1->3 : cheap (cost 2) but low safety (close to bad state 4)
    p.transitions.push_back({1,0,1,1.0, /*safety*/0.1, /*reliability*/1.0});
    p.transitions.push_back({2,1,3,1.0, 0.1, 1.0});
    // Path B: 0->2->3 : pricier (cost 4) but high safety (far from bad state 4)
    p.transitions.push_back({3,0,2,2.0, 0.95, 1.0});
    p.transitions.push_back({4,2,3,2.0, 0.95, 1.0});

    std::cout << "-- weighting cost only (safetyWeight=0) --\n";
    LPAStarPlanner cheapFirst;
    cheapFirst.safetyWeight = 0.0;
    auto r1 = cheapFirst.plan(p);
    printResult(r1);
    check(r1.statePath == std::vector<uint64_t>({0,1,3}), "cost-dominant planner picks cheap/close path");

    std::cout << "-- weighting safety heavily (safetyWeight=5) --\n";
    LPAStarPlanner safetyFirst;
    safetyFirst.safetyWeight = 5.0;
    auto r2 = safetyFirst.plan(p);
    printResult(r2);
    check(r2.statePath == std::vector<uint64_t>({0,2,3}), "safety-dominant planner picks farther/safer path");
    std::cout << "  Explanation: effective edge weight = cost*(1+relW*(1-reliability)) "
                 "+ safetyW*(1-safety). Raising safetyWeight increases the penalty for "
                 "low-safety edges until the safer, costlier path becomes cheaper overall. "
                 "This is a direct implementation of Score(P) = aG - bC + gD + dR: C and D "
                 "trade off through the safetyWeight scalarization coefficient.\n";
}

// ---------------------------------------------------------------------------
// Test Case 4: Dynamic transition -- edge (A,G) goes unavailable mid-flight,
// demonstrated via the INCREMENTAL interface (no full re-init).
// ---------------------------------------------------------------------------
void testCase4() {
    std::cout << "\n=== Test Case 4: Dynamic Transition (incremental) ===\n";
    PlanningProblem p;
    p.initialState = 0; p.goalState = 2; // S=0, A=1, G=2
    p.states = { {0,{0,0}}, {1,{1,0}}, {2,{2,0}} };
    p.transitions = {
        {1,0,1,1.0}, {2,1,2,1.0},   // S->A->G
        {3,0,1,1.0}                 // (unused duplicate, ignore)
    };
    p.transitions = { {1,0,1,1.0}, {2,1,2,1.0} };
    LPAStarPlanner planner;
    planner.loadProblem(p);
    auto r0 = planner.replan();
    printResult(r0);
    check(r0.statePath == std::vector<uint64_t>({0,1,2}), "initial path S->A->G found");

    // (A,G) becomes unavailable -> no path exists yet, incremental update only
    planner.setTransitionAvailable(2, false);
    auto r1 = planner.replan();
    printResult(r1);
    check(!r1.success, "planner correctly reports no path once (A,G) is down and no alternative exists yet");

    // add alternative route S->B->G (B=3) via new transitions, still incremental
    p.states.push_back({3, {1,1}});
    planner.addTransition({4,0,3,1.5});
    planner.addTransition({5,3,2,1.5});
    auto r2 = planner.replan();
    printResult(r2);
    check(r2.success && r2.statePath == std::vector<uint64_t>({0,3,2}), "alternative path found after adding S->B->G");
    std::cout << "  expanded on this incremental update: " << r2.statesExpanded
               << " (full replan from scratch would touch every vertex; LPA* only "
               << "reprocesses vertices whose rhs actually changed).\n";
}

// ---------------------------------------------------------------------------
// Test Case 5: Goal update mid-execution, no rebuild of g/rhs.
// ---------------------------------------------------------------------------
void testCase5() {
    std::cout << "\n=== Test Case 5: Goal Update ===\n";
    PlanningProblem p;
    p.initialState = 0; p.goalState = 2; // originally S(0)->A(1)->G_old(2)
    p.states = { {0,{0,0}}, {1,{1,0}}, {2,{2,0}}, {3,{0,2}} };
    p.transitions = { {1,0,1,1.0}, {2,1,2,1.0}, {3,0,3,1.0} };
    LPAStarPlanner planner;
    planner.loadProblem(p);
    auto r0 = planner.replan();
    printResult(r0);
    check(r0.success && r0.statePath.back() == 2, "initial plan reaches original goal (state 2)");

    planner.setGoal(3); // goal changes to state 3, reachable directly from S
    auto r1 = planner.replan();
    printResult(r1);
    check(r1.success && r1.statePath == std::vector<uint64_t>({0,3}), "revised path reaches NEW goal (state 3)");
    std::cout << "  Note: setGoal() reused all existing g/rhs values (no reinitialization); "
                 "only the goal-relative priority keys were recomputed for the open list, "
                 "per LPA* theory (rhs(s) depends solely on predecessors' g-values, never "
                 "on the goal).\n";
}

// ---------------------------------------------------------------------------
// Test Case 6: Transition addition -- a shortcut is inserted, planner must
// discover the improved (cheaper) solution incrementally.
// ---------------------------------------------------------------------------
void testCase6() {
    std::cout << "\n=== Test Case 6: Transition Addition (shortcut) ===\n";
    PlanningProblem p;
    p.initialState = 0; p.goalState = 3; // S=0 -> A=1 -> B=2 -> G=3, cost 3
    p.states = { {0,{0,0}}, {1,{1,0}}, {2,{2,0}}, {3,{3,0}} };
    p.transitions = { {1,0,1,1.0}, {2,1,2,1.0}, {3,2,3,1.0} };
    LPAStarPlanner planner;
    planner.loadProblem(p);
    auto r0 = planner.replan();
    printResult(r0);
    check(std::fabs(r0.totalCost - 3.0) < 1e-9, "baseline path cost 3.0 via S->A->B->G");

    planner.addTransition({4, 0, 3, 0.5}); // direct shortcut S->G, cost 0.5
    auto r1 = planner.replan();
    printResult(r1);
    check(r1.statePath == std::vector<uint64_t>({0,3}) && std::fabs(r1.totalCost - 0.5) < 1e-9,
          "shortcut S->G discovered and adopted (cost 0.5 < 3.0)");
}

int main() {
    std::cout << "LPA* Safe Semantic Planner -- Assignment 1 Verification Suite\n";
    std::cout << "================================================================\n";
    testCase1();
    testCase2();
    testCase3();
    testCase4();
    testCase5();
    testCase6();
    std::cout << "\n================================================================\n";
    std::cout << "TOTAL: " << g_pass << " passed, " << g_fail << " failed"
               << ((g_fail == 0) ? "  -> ALL CONDITIONS SATISFIED\n" : "  -> FAILURES PRESENT\n");
    return g_fail == 0 ? 0 : 1;
}
