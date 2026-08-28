#pragma once
#include "types.hpp"
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <algorithm>
#include <chrono>
#include <stdexcept>

// ---------------------------------------------------------------------------
// Priority key used by LPA*'s open list: k(s) = [ min(g,rhs)+h(s) ; min(g,rhs) ]
// Lexicographic ordering, per Koenig & Likhachev (2002/2004).
// ---------------------------------------------------------------------------
struct LPAKey {
    double k1;
    double k2;
    bool operator<(const LPAKey& o) const {
        if (std::fabs(k1 - o.k1) > 1e-9) return k1 < o.k1;
        return k2 < o.k2 - 1e-9;
    }
    bool operator>=(const LPAKey& o) const { return !(*this < o); }
};

class LPAStarPlanner : public Planner {
public:
    // Cost-shaping weights: how heavily reliability risk and safety deficit
    // penalize an edge's effective weight, on top of its raw `cost`.
    double reliabilityWeight = 1.0;
    double safetyWeight = 1.0;

    // ---- One-shot interface (Planner contract) --------------------------
    PlanningResult plan(const PlanningProblem& problem) override {
        loadProblem(problem);
        initialize();
        auto res = computeShortestPathAndExtract();
        return res;
    }

    // ---- Incremental / lifelong interface --------------------------------
    // Load a problem WITHOUT resetting g/rhs if the graph topology (state
    // ids) is identical to what is already loaded -- this is what allows
    // LPA* to reuse prior search effort. Call loadProblem() once, then use
    // the mutators below for updates.
    void loadProblem(const PlanningProblem& problem) {
        problem_ = problem;
        rebuildAdjacency();
        if (!initializedOnce_) {
            initialize();
            initializedOnce_ = true;
        }
    }

    // Test Case 2 / bad-state avoidance: mark a state as bad. All incident
    // transitions become unavailable and affected vertices are re-evaluated.
    void setBadState(uint64_t stateId) {
        badStates_.insert(stateId);
        for (auto& t : transitionsById_) {
            if (t.second.from == stateId || t.second.to == stateId) {
                updateVertex(t.second.to);
            }
        }
    }

    // Test Case 4: transition availability change (e.g. edge goes down).
    void setTransitionAvailable(uint64_t transitionId, bool available) {
        auto it = transitionsById_.find(transitionId);
        if (it == transitionsById_.end()) throw std::runtime_error("unknown transition id");
        it->second.available = available;
        updateVertex(it->second.to);
    }

    // Edge cost change (generic LPA* incremental-update entry point).
    void updateTransitionCost(uint64_t transitionId, double newCost) {
        auto it = transitionsById_.find(transitionId);
        if (it == transitionsById_.end()) throw std::runtime_error("unknown transition id");
        it->second.cost = newCost;
        updateVertex(it->second.to);
    }

    // Test Case 6: a brand-new transition is inserted.
    void addTransition(const Transition& t) {
        transitionsById_[t.id] = t;
        successors_[t.from].push_back(t.id);
        predecessors_[t.to].push_back(t.id);
        updateVertex(t.to);
    }

    // Test Case 5: goal changes. Because LPA* computes g/rhs from the START
    // outward (rhs(s) depends only on predecessors' g-values, never on the
    // goal), g and rhs stay valid across a goal change. Only the heuristic
    // key (which is goal-relative) is stale, so we re-key every vertex
    // currently in the open list and continue the search from there --
    // no reinitialization of g/rhs needed.
    void setGoal(uint64_t newGoal) {
        problem_.goalState = newGoal;
        std::set<std::pair<LPAKey, uint64_t>> reheaped;
        for (auto& kv : keyOfOpen_) {
            reheaped.insert({calculateKey(kv.first), kv.first});
        }
        open_ = std::move(reheaped);
        keyOfOpen_.clear();
        for (auto& pr : open_) keyOfOpen_[pr.second] = pr.first;
    }

    // Runs (or resumes) ComputeShortestPath and extracts the path to the
    // CURRENT goal. Call after any of the mutators above for O(affected)
    // replanning instead of a full from-scratch search.
    PlanningResult replan() {
        return computeShortestPathAndExtract();
    }

    long lastStatesExpanded() const { return statesExpanded_; }

    // Tunes heuristic aggressiveness: h(s) = euclid(s,goal) * scale.
    // scale=0 (default) = Dijkstra-equivalent, always admissible/consistent.
    // scale>0 speeds up search but MUST be <= min effective edge weight per
    // unit Euclidean distance in the embedding, or admissibility (and thus
    // optimality) breaks. Left conservative by default; caller opts in.
    void setHeuristicScale(double scale) { minCostPerDist_ = scale; }

private:
    PlanningProblem problem_;
    std::unordered_map<uint64_t, Transition> transitionsById_;
    std::unordered_map<uint64_t, std::vector<uint64_t>> successors_;   // stateId -> transition ids out
    std::unordered_map<uint64_t, std::vector<uint64_t>> predecessors_; // stateId -> transition ids in
    std::unordered_map<uint64_t, State> stateById_;
    std::unordered_set<uint64_t> badStates_;

    std::unordered_map<uint64_t, double> g_, rhs_;
    std::set<std::pair<LPAKey, uint64_t>> open_;
    std::unordered_map<uint64_t, LPAKey> keyOfOpen_;
    long statesExpanded_ = 0;
    bool initializedOnce_ = false;

    void rebuildAdjacency() {
        transitionsById_.clear();
        successors_.clear();
        predecessors_.clear();
        stateById_.clear();
        badStates_.clear();
        for (auto& s : problem_.states) stateById_[s.id] = s;
        for (auto b : problem_.badStates) badStates_.insert(b);
        for (auto& t : problem_.transitions) {
            transitionsById_[t.id] = t;
            successors_[t.from].push_back(t.id);
            predecessors_[t.to].push_back(t.id);
        }
    }

    bool edgeUsable(const Transition& t) const {
        return t.available && !badStates_.count(t.from) && !badStates_.count(t.to);
    }

    // Effective edge weight folds in reliability risk and safety deficit
    // on top of raw cost, per assignment Score(P) = aG - bC + gD + dR:
    // we want low cost, high safety, high reliability -> low effective weight.
    double edgeWeight(const Transition& t) const {
        double riskFactor = 1.0 + reliabilityWeight * (1.0 - t.reliability);
        double safetyPenalty = safetyWeight * (1.0 - t.safety);
        return t.cost * riskFactor + safetyPenalty;
    }

    double heuristic(uint64_t s) const {
        auto itS = stateById_.find(s);
        auto itG = stateById_.find(problem_.goalState);
        if (itS == stateById_.end() || itG == stateById_.end()) return 0.0;
        // Admissible provided min effective edge weight per unit Euclidean
        // distance >= 1; scaled by minCostPerDist_ to preserve consistency
        // under the cost-shaping above.
        return euclidean(itS->second.embedding, itG->second.embedding) * minCostPerDist_;
    }
    double minCostPerDist_ = 0.0; // conservative (never overestimates) -> set to 0 by default (Dijkstra-equivalent, always admissible)

    double getG(uint64_t s) const { auto it = g_.find(s); return it == g_.end() ? INF : it->second; }
    double getRhs(uint64_t s) const { auto it = rhs_.find(s); return it == rhs_.end() ? INF : it->second; }
    static constexpr double INF = std::numeric_limits<double>::infinity();

    LPAKey calculateKey(uint64_t s) const {
        double m = std::min(getG(s), getRhs(s));
        return LPAKey{ m + heuristic(s), m };
    }

    void openInsert(uint64_t s) {
        LPAKey k = calculateKey(s);
        open_.insert({k, s});
        keyOfOpen_[s] = k;
    }
    void openRemove(uint64_t s) {
        auto it = keyOfOpen_.find(s);
        if (it == keyOfOpen_.end()) return;
        open_.erase({it->second, s});
        keyOfOpen_.erase(it);
    }
    bool openContains(uint64_t s) const { return keyOfOpen_.count(s) != 0; }

    void initialize() {
        g_.clear(); rhs_.clear(); open_.clear(); keyOfOpen_.clear();
        statesExpanded_ = 0;
        rhs_[problem_.initialState] = 0.0;
        openInsert(problem_.initialState);
    }

    void updateVertex(uint64_t u) {
        if (u != problem_.initialState) {
            double best = INF;
            auto it = predecessors_.find(u);
            if (it != predecessors_.end()) {
                for (auto tid : it->second) {
                    const Transition& t = transitionsById_[tid];
                    if (!edgeUsable(t)) continue;
                    double cand = getG(t.from) + edgeWeight(t);
                    if (cand < best) best = cand;
                }
            }
            rhs_[u] = best;
        }
        openRemove(u);
        if (std::fabs(getG(u) - getRhs(u)) > 1e-12) openInsert(u);
    }

    void computeShortestPath() {
        while (!open_.empty() &&
               ( open_.begin()->first < calculateKey(problem_.goalState) ||
                 std::fabs(getRhs(problem_.goalState) - getG(problem_.goalState)) > 1e-12 )) {
            uint64_t u = open_.begin()->second;
            statesExpanded_++;
            LPAKey kOld = open_.begin()->first;
            LPAKey kNew = calculateKey(u);
            if (kOld < kNew) {
                openRemove(u); openInsert(u); // re-key (lazy consistency), do not expand yet
                continue;
            }
            if (getG(u) > getRhs(u)) {
                g_[u] = getRhs(u);
                openRemove(u);
                auto it = successors_.find(u);
                if (it != successors_.end())
                    for (auto tid : it->second) {
                        const Transition& t = transitionsById_[tid];
                        if (edgeUsable(t)) updateVertex(t.to);
                    }
            } else {
                g_[u] = INF;
                updateVertex(u);
                auto it = successors_.find(u);
                if (it != successors_.end())
                    for (auto tid : it->second) {
                        const Transition& t = transitionsById_[tid];
                        if (edgeUsable(t)) updateVertex(t.to);
                    }
            }
        }
    }

    PlanningResult computeShortestPathAndExtract() {
        auto t0 = std::chrono::high_resolution_clock::now();
        computeShortestPath();
        auto t1 = std::chrono::high_resolution_clock::now();

        PlanningResult res;
        res.statesExpanded = statesExpanded_;
        res.planningTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

        double gGoal = getG(problem_.goalState);
        if (gGoal >= INF) {
            res.success = false;
            return res;
        }

        // Backtrack via predecessor with min g(pred)+w(pred,cur), matching rhs.
        std::vector<uint64_t> statePath, transitionPath;
        uint64_t cur = problem_.goalState;
        statePath.push_back(cur);
        int guard = 0;
        while (cur != problem_.initialState) {
            if (++guard > (int)stateById_.size() + 5) throw std::runtime_error("path extraction failed to converge");
            auto it = predecessors_.find(cur);
            double best = INF; uint64_t bestPred = 0; uint64_t bestTid = 0;
            if (it != predecessors_.end()) {
                for (auto tid : it->second) {
                    const Transition& t = transitionsById_[tid];
                    if (!edgeUsable(t)) continue;
                    double cand = getG(t.from) + edgeWeight(t);
                    if (cand < best - 1e-9) { best = cand; bestPred = t.from; bestTid = tid; }
                }
            }
            if (best >= INF) { res.success = false; return res; }
            transitionPath.push_back(bestTid);
            statePath.push_back(bestPred);
            cur = bestPred;
        }
        std::reverse(statePath.begin(), statePath.end());
        std::reverse(transitionPath.begin(), transitionPath.end());

        double totalCost = 0.0, minDistToBad = INF;
        for (auto tid : transitionPath) totalCost += transitionsById_[tid].cost;
        for (auto sid : statePath) {
            auto itS = stateById_.find(sid);
            if (itS == stateById_.end()) continue;
            for (auto bId : badStates_) {
                auto itB = stateById_.find(bId);
                if (itB == stateById_.end()) continue;
                double d = euclidean(itS->second.embedding, itB->second.embedding);
                if (d < minDistToBad) minDistToBad = d;
            }
        }
        if (minDistToBad >= INF) minDistToBad = 0.0; // no bad states defined

        res.success = true;
        res.statePath = statePath;
        res.transitionPath = transitionPath;
        res.totalCost = totalCost;
        res.safetyScore = minDistToBad;
        return res;
    }
};
