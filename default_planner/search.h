
#ifndef search_hpp
#define search_hpp

#include "Types.h"
#include "utils.h"
#include "Memory.h"
#include "heap.h"
#include "search_node.h"
#include "heuristics.h"

namespace DefaultPlanner{
//a astar minimized the opposide traffic flow with existing traffic flow


s_node astar(SharedEnvironment* env, std::vector<Int4>& flow,
    HeuristicTable& ht, Traj& traj,
    MemoryPool& mem, int start, int goal, Neighbors* ns);


    // constraints table added version
    s_node astar(SharedEnvironment* env, std::vector<std::vector<std::pair<bool, std::array<bool, 4>>>>& constraint_flow,
                 std::vector<Int4>& flow,
                 HeuristicTable& ht, std::vector<HeuristicTable>& all_ht, Traj& traj,
                 MemoryPool& mem, int start, int goal, Neighbors* ns);



    // simulation A*
    int simulation_astar(SharedEnvironment* env, std::vector<std::vector<std::pair<bool, std::array<bool, 4>>>>& constraint_flow,
                 std::vector<Int4>& flow, HeuristicTable& ht,  HeuristicTable& temp_ht,
                 Traj& traj, MemoryPool& mem, int start, int temp_goal,
                 int goal, int start_g, Neighbors* ns);
}

#endif