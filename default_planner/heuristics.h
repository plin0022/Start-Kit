
#ifndef heuristics_hpp
#define heuristics_hpp

#include "Types.h"
#include "utils.h"
#include <queue>
#include "TrajLNS.h"
#include "search_node.h"


//class TrajLNS;


namespace DefaultPlanner{

void init_heuristics(SharedEnvironment* env);

void init_neighbor(SharedEnvironment* env);

void init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location);

void init_flextable(HeuristicTable& ht, SharedEnvironment* env, int goal_location);

int get_heuristic(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns);



// traffic

void init_traffic_heuristic(FlowHeuristic& ht, SharedEnvironment* env, int goal, int start);


float get_traffic_heuristic(TrajLNS& lns, FlowHeuristic& ht, SharedEnvironment* env,
                          int source, int start, Neighbors* ns);

//

uint get_flex(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns);

uint get_flex(TrajLNS& lns, int ai, int source, Neighbors* ns);

int get_h(SharedEnvironment* env, int source, int target);


void init_dist_2_path(Dist2Path& dp, SharedEnvironment* env, Traj& path);

std::pair<int,int> get_source_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns);

int get_dist_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns);

}
#endif