#ifndef TRAJ_LNS_H
#define TRAJ_LNS_H

#include "Types.h"
#include "Memory.h"
#include "search_node.h"
#include "heap.h"
//#include "heuristics.h"
#include <iostream>

#include <set>

namespace DefaultPlanner{
// enum ADAPTIVE {RANDOM, CONGESTION, COUNT};
enum ADAPTIVE {RANDOM, CONGESTION, DEVIATION, COUNT};

extern std::vector<HeuristicTable> global_heuristictable;
extern Neighbors global_neighbors;

struct FW_Metric{
    int id;
    int deviation;
    int last_replan_t;
    int rand;

    FW_Metric(int i, int d, int l) : id(i), deviation(d),last_replan_t(l){};
    FW_Metric(){};
};

struct FlowHeuristic{
    int target;
    int origin;
    std::vector<float> htable;  // record the h_value

    pqueue<s_node,cmp_less_of_123, min_q> open;

//    pqueue_min_of open;       // priority queue
    MemoryPool mem;




    bool empty(){
        return htable.empty() && open.empty();
    }

    void reset(){
        htable.clear();
        open.clear();
        mem.reset();
    }

};

class TrajLNS{
    public:
    SharedEnvironment* env;
    std::vector<int> tasks;

    std::vector<int> start_locs;

    TimePoint start_time;
    int t_ms=0;

    std::vector<Traj> trajs;

    std::vector<MDD_Traj> mdd_trajs;

    std::vector<std::pair<int,int>> deviation_agents;

    std::vector<Int4> flow;

    std::vector<Float4> float_flow;

    std::vector<HeuristicTable>& heuristics;

    std::vector<FlowHeuristic> flow_heuristics;

    std::vector<THeuristicTable> t_heuristics;

    std::vector<Dist2Path> traj_dists;
    std::vector<s_node> goal_nodes;// store the goal node of single agent search for each agent. contains all cost information.

    std::vector<FW_Metric> fw_metrics;
    Neighbors& neighbors;


    int traj_inited = 0;
    int dist2path_inited = 0;
    int soc = 0;

    MemoryPool mem;

    void init_mem(){
        mem.init(env->map.size());
    }

    void init_all_mem()
    {
        for (int i = 0; i < env->num_of_agents; i++)
        {
            flow_heuristics[i].mem.init(env->map.size());
        }
    }

    TrajLNS(SharedEnvironment* env, std::vector<HeuristicTable>& heuristics, Neighbors& neighbors):
        env(env),
        trajs(env->num_of_agents),
        mdd_trajs(env->num_of_agents),
        tasks(env->num_of_agents),
        start_locs(env->num_of_agents),
        flow(env->map.size(),Int4({0,0,0,0})),
        float_flow(env->map.size(), Float4({0,0,0,0})),
        heuristics(heuristics),
        flow_heuristics(env->num_of_agents),
        t_heuristics(env->num_of_agents),
        traj_dists(env->num_of_agents),goal_nodes(env->num_of_agents),
        fw_metrics(env->num_of_agents),neighbors(neighbors){
        };


    TrajLNS():heuristics(global_heuristictable), neighbors(global_neighbors){};

    

};
}
#endif