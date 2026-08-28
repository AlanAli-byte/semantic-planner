#include "../include/lpastar_planner.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <fstream>

// ---------------------------------------------------------------------------
// Interactive Safe Semantic Planner CLI.
// Reads commands from stdin (typed, piped, or redirected from a file) and
// builds a PlanningProblem live, then drives an LPAStarPlanner against it.
//
// Commands:
//   STATE <id> <x1> <x2> ... <xd>          define/redefine a state
//   TRANS <id> <from> <to> <cost> [safety] [reliability] [available:0/1]
//                                          define/redefine a transition
//   INIT <id>                              set initial state
//   GOAL <id>                              set goal state
//   BAD  <id>                              mark a state as bad (hard-avoid)
//   LOAD                                    finalize problem, load into planner
//   PLAN                                    cold-start solve, print result
//   REPLAN                                  incremental solve (after mutators)
//   SETBAD <id>                            mark bad state AFTER loading (incremental)
//   SETAVAIL <transId> <0|1>               toggle transition availability (incremental)
//   SETCOST <transId> <cost>               change a transition's cost (incremental)
//   ADDTRANS <id> <from> <to> <cost> [safety] [reliability]
//                                          add a new transition (incremental)
//   SETGOAL <id>                           change goal (incremental)
//   WEIGHTS <safetyWeight> <reliabilityWeight>
//                                          set scalarization weights
//   SHOW                                    print current problem summary
//   HELP                                    show this text
//   EXIT / QUIT                            leave
//
// Nothing is assumed silently: malformed input is rejected with a clear
// reason and the state/problem is left unchanged.
// ---------------------------------------------------------------------------

static void printHelp() {
    std::cout <<
R"(Commands:
  STATE <id> <x1> <x2> ... <xd>          define/redefine a state (embedding coords)
  TRANS <id> <from> <to> <cost> [safety] [reliability] [available:0|1]
  INIT <id>                              set initial state id
  GOAL <id>                              set goal state id
  BAD  <id>                              mark a state id as bad (pre-load)
  LOAD                                    finalize + load problem into planner
  PLAN                                    cold-start solve, print result
  REPLAN                                  incremental solve (call after mutators below)
  SETBAD <id>                            mark bad state (post-load, incremental)
  SETAVAIL <transId> <0|1>               toggle transition availability (incremental)
  SETCOST <transId> <cost>               change transition cost (incremental)
  ADDTRANS <id> <from> <to> <cost> [safety] [reliability]
  SETGOAL <id>                           change goal (incremental)
  WEIGHTS <safetyWeight> <reliabilityWeight>
  SHOW                                    print current problem summary
  HELP                                    this text
  EXIT / QUIT
Lines starting with # are comments. Blank lines are ignored.
)";
}

static void printResult(const PlanningResult& r) {
    if (!r.success) {
        std::cout << "RESULT: no path found (goal unreachable without visiting a bad state, "
                     "or no route exists with current transitions).\n";
        std::cout << "  states expanded: " << r.statesExpanded
                   << "  time: " << r.planningTimeMs << "ms\n";
        return;
    }
    std::cout << "RESULT: success\n  path: ";
    for (size_t i = 0; i < r.statePath.size(); ++i) {
        std::cout << r.statePath[i];
        if (i + 1 < r.statePath.size()) std::cout << " -> ";
    }
    std::cout << "\n  transitions used: ";
    for (size_t i = 0; i < r.transitionPath.size(); ++i) {
        std::cout << r.transitionPath[i];
        if (i + 1 < r.transitionPath.size()) std::cout << ", ";
    }
    std::cout << "\n  total cost: " << r.totalCost
               << "\n  min distance to nearest bad state along path: " << r.safetyScore
               << "\n  states expanded: " << r.statesExpanded
               << "\n  planning time: " << r.planningTimeMs << "ms\n";
}

int main(int argc, char** argv) {
    std::cout << "Safe Semantic Planner (LPA*) — interactive mode.\n"
                 "Type HELP for commands, EXIT to quit.\n";

    // Optional: read a script file instead of interactive stdin.
    std::istream* in = &std::cin;
    std::ifstream fileIn;
    if (argc > 1) {
        fileIn.open(argv[1]);
        if (!fileIn) {
            std::cerr << "ERROR: could not open file '" << argv[1] << "'\n";
            return 1;
        }
        in = &fileIn;
        std::cout << "(reading commands from " << argv[1] << ")\n";
    }

    PlanningProblem problem;
    std::map<uint64_t, State> stateMap;
    std::map<uint64_t, Transition> transMap;
    bool hasInit = false, hasGoal = false;
    LPAStarPlanner planner;
    bool loaded = false;

    auto rebuildProblemVectors = [&]() {
        problem.states.clear();
        for (auto& kv : stateMap) problem.states.push_back(kv.second);
        problem.transitions.clear();
        for (auto& kv : transMap) problem.transitions.push_back(kv.second);
    };

    std::string line;
    while (std::getline(*in, line)) {
        // strip comments / trim
        auto hashPos = line.find('#');
        if (hashPos != std::string::npos) line = line.substr(0, hashPos);
        std::istringstream iss(line);
        std::string cmd;
        if (!(iss >> cmd)) continue; // blank line

        if (in != &std::cin) std::cout << "> " << line << "\n"; // echo when scripted

        try {
            if (cmd == "HELP") {
                printHelp();
            } else if (cmd == "EXIT" || cmd == "QUIT") {
                break;
            } else if (cmd == "STATE") {
                uint64_t id; if (!(iss >> id)) throw std::runtime_error("STATE needs an id");
                std::vector<double> emb; double v;
                while (iss >> v) emb.push_back(v);
                if (emb.empty()) throw std::runtime_error("STATE needs at least one coordinate");
                stateMap[id] = State(id, emb);
                std::cout << "OK: state " << id << " defined with " << emb.size() << "D embedding\n";
            } else if (cmd == "TRANS") {
                uint64_t id, from, to; double cost;
                if (!(iss >> id >> from >> to >> cost))
                    throw std::runtime_error("TRANS needs: id from to cost [safety] [reliability] [available]");
                double safety = 1.0, reliability = 1.0; int avail = 1;
                if (iss >> safety) { if (!(iss >> reliability)) reliability = 1.0; }
                if (iss >> avail) {}
                if (cost < 0) throw std::runtime_error("cost must be >= 0");
                if (safety < 0 || safety > 1) throw std::runtime_error("safety must be in [0,1]");
                if (reliability < 0 || reliability > 1) throw std::runtime_error("reliability must be in [0,1]");
                transMap[id] = Transition(id, from, to, cost, safety, reliability, avail != 0);
                std::cout << "OK: transition " << id << " (" << from << "->" << to << ") defined\n";
            } else if (cmd == "INIT") {
                uint64_t id; if (!(iss >> id)) throw std::runtime_error("INIT needs an id");
                problem.initialState = id; hasInit = true;
                std::cout << "OK: initial state = " << id << "\n";
            } else if (cmd == "GOAL") {
                uint64_t id; if (!(iss >> id)) throw std::runtime_error("GOAL needs an id");
                problem.goalState = id; hasGoal = true;
                std::cout << "OK: goal state = " << id << "\n";
            } else if (cmd == "BAD") {
                uint64_t id; if (!(iss >> id)) throw std::runtime_error("BAD needs an id");
                problem.badStates.push_back(id);
                std::cout << "OK: state " << id << " marked bad (pre-load)\n";
            } else if (cmd == "LOAD") {
                if (!hasInit || !hasGoal) throw std::runtime_error("set INIT and GOAL before LOAD");
                if (stateMap.find(problem.initialState) == stateMap.end())
                    throw std::runtime_error("initial state id not defined via STATE");
                if (stateMap.find(problem.goalState) == stateMap.end())
                    throw std::runtime_error("goal state id not defined via STATE");
                rebuildProblemVectors();
                planner.loadProblem(problem);
                loaded = true;
                std::cout << "OK: problem loaded (" << problem.states.size() << " states, "
                           << problem.transitions.size() << " transitions, "
                           << problem.badStates.size() << " bad states)\n";
            } else if (cmd == "PLAN") {
                if (!loaded) throw std::runtime_error("LOAD the problem first");
                auto r = planner.plan(problem); // cold start
                printResult(r);
            } else if (cmd == "REPLAN") {
                if (!loaded) throw std::runtime_error("LOAD the problem first");
                auto r = planner.replan();
                printResult(r);
            } else if (cmd == "SETBAD") {
                if (!loaded) throw std::runtime_error("LOAD the problem first");
                uint64_t id; if (!(iss >> id)) throw std::runtime_error("SETBAD needs an id");
                planner.setBadState(id);
                std::cout << "OK: state " << id << " marked bad (incremental). Run REPLAN to see effect.\n";
            } else if (cmd == "SETAVAIL") {
                if (!loaded) throw std::runtime_error("LOAD the problem first");
                uint64_t tid; int avail;
                if (!(iss >> tid >> avail)) throw std::runtime_error("SETAVAIL needs: transId 0|1");
                planner.setTransitionAvailable(tid, avail != 0);
                std::cout << "OK: transition " << tid << " availability = " << avail
                           << " (incremental). Run REPLAN.\n";
            } else if (cmd == "SETCOST") {
                if (!loaded) throw std::runtime_error("LOAD the problem first");
                uint64_t tid; double cost;
                if (!(iss >> tid >> cost)) throw std::runtime_error("SETCOST needs: transId cost");
                if (cost < 0) throw std::runtime_error("cost must be >= 0");
                planner.updateTransitionCost(tid, cost);
                std::cout << "OK: transition " << tid << " cost = " << cost << " (incremental). Run REPLAN.\n";
            } else if (cmd == "ADDTRANS") {
                if (!loaded) throw std::runtime_error("LOAD the problem first");
                uint64_t id, from, to; double cost;
                if (!(iss >> id >> from >> to >> cost))
                    throw std::runtime_error("ADDTRANS needs: id from to cost [safety] [reliability]");
                double safety = 1.0, reliability = 1.0;
                if (iss >> safety) { if (!(iss >> reliability)) reliability = 1.0; }
                Transition t(id, from, to, cost, safety, reliability, true);
                transMap[id] = t;
                planner.addTransition(t);
                std::cout << "OK: transition " << id << " (" << from << "->" << to
                           << ") added (incremental). Run REPLAN.\n";
            } else if (cmd == "SETGOAL") {
                if (!loaded) throw std::runtime_error("LOAD the problem first");
                uint64_t id; if (!(iss >> id)) throw std::runtime_error("SETGOAL needs an id");
                if (stateMap.find(id) == stateMap.end())
                    throw std::runtime_error("goal state id not defined via STATE");
                planner.setGoal(id);
                problem.goalState = id;
                std::cout << "OK: goal changed to " << id << " (incremental, g/rhs reused). Run REPLAN.\n";
            } else if (cmd == "WEIGHTS") {
                double sw, rw;
                if (!(iss >> sw >> rw)) throw std::runtime_error("WEIGHTS needs: safetyWeight reliabilityWeight");
                planner.safetyWeight = sw;
                planner.reliabilityWeight = rw;
                std::cout << "OK: safetyWeight=" << sw << " reliabilityWeight=" << rw
                           << ". Weights only affect the NEXT cold-start PLAN.\n";
            } else if (cmd == "SHOW") {
                std::cout << "states: " << stateMap.size() << "  transitions: " << transMap.size()
                           << "\ninit: " << (hasInit ? std::to_string(problem.initialState) : "(unset)")
                           << "  goal: " << (hasGoal ? std::to_string(problem.goalState) : "(unset)")
                           << "\nbad states (pre-load list): ";
                for (auto b : problem.badStates) std::cout << b << " ";
                std::cout << "\nloaded: " << (loaded ? "yes" : "no") << "\n";
            } else {
                std::cout << "ERROR: unknown command '" << cmd << "'. Type HELP.\n";
            }
        } catch (const std::exception& e) {
            std::cout << "ERROR: " << e.what() << "\n";
        }
    }

    std::cout << "Goodbye.\n";
    return 0;
}
