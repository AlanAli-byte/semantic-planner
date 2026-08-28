#pragma once
#include <cstdint>
#include <vector>
#include <cmath>
#include <limits>

// ---------------------------------------------------------------------------
// State: a point in the finite Cartesian state space R^d.
// ---------------------------------------------------------------------------
class State {
public:
    uint64_t id;
    std::vector<double> embedding;

    State() : id(0) {}
    State(uint64_t id_, std::vector<double> emb) : id(id_), embedding(std::move(emb)) {}
};

// ---------------------------------------------------------------------------
// Transition: directed edge s_from -> s_to with cost / safety / reliability
// / availability attributes.
// ---------------------------------------------------------------------------
class Transition {
public:
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;        // 0 (unsafe) .. 1 (safe)
    double reliability;    // 0 (unreliable) .. 1 (fully reliable)
    bool available;

    Transition() : id(0), from(0), to(0), cost(0), safety(1.0),
                   reliability(1.0), available(true) {}

    Transition(uint64_t id_, uint64_t f, uint64_t t, double c,
               double s = 1.0, double r = 1.0, bool avail = true)
        : id(id_), from(f), to(t), cost(c), safety(s),
          reliability(r), available(avail) {}
};

// ---------------------------------------------------------------------------
// PlanningProblem: the full problem instance handed to a Planner.
// ---------------------------------------------------------------------------
class PlanningProblem {
public:
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;
};

// ---------------------------------------------------------------------------
// PlanningResult: what a Planner returns.
// ---------------------------------------------------------------------------
class PlanningResult {
public:
    bool success = false;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost = std::numeric_limits<double>::infinity();
    double safetyScore = 0.0;      // min Euclidean distance to nearest bad state along path
    long statesExpanded = 0;
    double planningTimeMs = 0.0;
};

// ---------------------------------------------------------------------------
// Planner: abstract interface (per assignment spec).
// ---------------------------------------------------------------------------
class Planner {
public:
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
    virtual ~Planner() = default;
};

inline double euclidean(const std::vector<double>& a, const std::vector<double>& b) {
    double sum = 0.0;
    size_t n = std::min(a.size(), b.size());
    for (size_t i = 0; i < n; ++i) {
        double d = a[i] - b[i];
        sum += d * d;
    }
    return std::sqrt(sum);
}
