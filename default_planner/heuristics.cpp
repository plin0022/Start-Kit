
#include "heuristics.h"
#include <queue>
#include "pibt.h"

namespace DefaultPlanner{

std::vector<HeuristicTable> global_heuristictable;
Neighbors global_neighbors;



void init_neighbor(SharedEnvironment* env){
	global_neighbors.resize(env->rows * env->cols);
	for (int row=0; row<env->rows; row++){
		for (int col=0; col<env->cols; col++){
			int loc = row*env->cols+col;
			if (env->map[loc]==0){
				if (row>0 && env->map[loc-env->cols]==0){
					global_neighbors[loc].push_back(loc-env->cols);
				}
				if (row<env->rows-1 && env->map[loc+env->cols]==0){
					global_neighbors[loc].push_back(loc+env->cols);
				}
				if (col>0 && env->map[loc-1]==0){
					global_neighbors[loc].push_back(loc-1);
				}
				if (col<env->cols-1 && env->map[loc+1]==0){
					global_neighbors[loc].push_back(loc+1);
				}
			}
		}
	}
};

void init_heuristics(SharedEnvironment* env){
	if (global_heuristictable.size()==0){
		global_heuristictable.resize(env->map.size());
		init_neighbor(env);
	}

}

void init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location){
	// initialize my_heuristic, but have error on malloc: Region cookie corrupted for region
	ht.htable.clear();
	ht.htable.resize(env->map.size(),MAX_TIMESTEP);
	ht.open.clear();
	// generate a open that can save nodes (and a open_handle)
	HNode root(goal_location, 0, 0);
	ht.htable[goal_location] = 0;
	ht.open.push_back(root);  // add root to open
}


int get_heuristic(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns){
    if (ht.htable[source] < MAX_TIMESTEP) return ht.htable[source];

    std::vector<int> neighbors;
    int cost, diff;
    while (!ht.open.empty())
    {
        HNode curr = ht.open.front();
        ht.open.pop_front();


        getNeighborLocs(ns,neighbors,curr.location);


        for (int next : neighbors)
        {
            cost = curr.g + 1;

            assert(next >= 0 && next < env->map.size());
            //set current cost for reversed direction

            if (cost >= ht.htable[next] )
                continue;

            ht.open.emplace_back(next, cost, 0);
            ht.htable[next] = cost;

        }

        if (source == curr.location)
            return curr.g;
    }


    return MAX_TIMESTEP;
}



// traffic heuristic

void init_traffic_heuristic(FlowHeuristic& ht, SharedEnvironment* env, int goal, int start)
{
    // initialize tables
    ht.htable.resize(env->map.size(),MAX_TIMESTEP);


    // generate an open that can save nodes
    int h = manhattanDistance(start, goal, env);
    s_node *root = ht.mem.generate_node(goal, 0, h, 0, 0, 0);
    ht.htable[goal] = 0;
    ht.open.push(root);
}


// Reverse Resumable A*
// https://doi.org/10.1609/aiide.v1i1.18726
float get_traffic_heuristic(TrajLNS& lns, FlowHeuristic& ht, SharedEnvironment* env,
                          int source, int start, Neighbors* ns)
{
    if (ht.mem.has_node(source) && ht.mem.get_node(source)->is_closed())
        return ht.htable[source];

    large_than_fmin large;

    std::vector<int> neighbors;
    int h, diff, d;
    float cost, temp_op, temp_vertex;


    while (!ht.open.empty())
    {
        s_node *curr = ht.open.pop();
        assert(ht.htable[curr->id] == curr->g);
        ht.htable[curr->id] = curr->g;
        curr->close();


        getNeighborLocs(ns,neighbors, curr->id);
        for (int next : neighbors)
        {
            diff = curr->id - next;
            d = get_d(diff, env);

            temp_op = ((lns.float_flow[next].d[d] + 1) *
                       lns.float_flow[curr->id].d[(d + 2) % 4]);

            //all vertex flow
            //the sum of all out going edge flow is the same as the total number of vertex visiting.
            temp_vertex = 1;
            for (int j = 0; j < 4; j++) {
                temp_vertex += lns.float_flow[curr->id].d[j];
            }


            //set current cost for reversed direction
            cost = curr->g + 1 + temp_op + (temp_vertex - 1) / 2;

            h = manhattanDistance(next, start, env);

//            if (ht.empty())
//                h = manhattanDistance(next, start, env);
//            else
//                h = get_heuristic(ht, env, next, ns);


            assert(next >= 0 && next < env->map.size());


            if (!ht.mem.has_node(next))
            {
                s_node *next_node = ht.mem.generate_node(next, cost, h, 0, 0, 0);
                ht.open.push(next_node);
                ht.htable[next] = cost;
            }
            else
            {
                s_node *existing = ht.mem.get_node(next);
                assert(existing->g == ht.htable[next]);

                if (!existing->is_closed())
                {
                    if (large(*existing, cost))
                    {
                        existing->g = cost;
                        ht.htable[next] = cost;
                        ht.open.decrease_key(existing);
                    }
                }
                else
                {
                    if (large(*existing, cost))
                    {
                        std::cout << "error in astar: re-expansion" << std::endl;
                        assert(false);
                        exit(1);
                    }
                }
            }

        }

        if (source == curr->id)
            return curr->g;

    }

    assert(false);
    return MAX_TIMESTEP;
}



//// Reverse Resumable A*
//// https://doi.org/10.1609/aiide.v1i1.18726
//int get_traffic_heuristic(TrajLNS& lns, THeuristicTable& ht, SharedEnvironment* env,
//                          int source, int start, Neighbors* ns)
//{
//    if (ht.traffic_closed[source]) return ht.traffic_htable[source];
//
//    std::vector<int> neighbors;
//    int cost, diff, d, temp_op, temp_vertex, curr_location, curr_g;
//
//    while (!ht.traffic_open.empty())
//    {
//        // free memory and remove the entry in map for a closed node
//        HNode *curr = ht.traffic_open.pop();
//        curr_location = curr->location;
//        curr_g = curr->g;
//        ht.traffic_htable[curr_location] = curr_g;
//        ht.traffic_closed[curr_location] = true;
//        delete curr;
//        ht.node_list.erase(curr_location);
//
//
//        getNeighborLocs(ns,neighbors, curr_location);
//
//
//        for (int next : neighbors)
//        {
//            diff = curr_location - next;
//            d = get_d(diff, env);
//
//            temp_op = ((lns.flow[next].d[d] + 1) *
//                       lns.flow[curr_location].d[(d + 2) % 4]);
//
//            //all vertex flow
//            //the sum of all out going edge flow is the same as the total number of vertex visiting.
//            temp_vertex = 1;
//            for (int j = 0; j < 4; j++) {
//                temp_vertex += lns.flow[curr_location].d[j];
//            }
//
//            //set current cost for reversed direction
//
//            cost = curr_g + 1 + temp_op + (temp_vertex - 1) / 2;
//
//
//            assert(next >= 0 && next < env->map.size());
//
//
//            if (ht.traffic_htable[next] == MAX_TIMESTEP)
//            {
//                HNode* next_node = new HNode(next, cost, manhattanDistance(start, next, env));
//                ht.traffic_open.push(next_node);
//                ht.traffic_htable[next] = cost;
//                ht.node_list[next] = next_node;
//            }
//            else
//            {
//                if (!ht.traffic_closed[next])
//                {
//                    if (cost < ht.traffic_htable[next])
//                    {
//                        HNode* existing = ht.node_list[next];
//                        existing->g = cost;
//                        ht.traffic_htable[next] = cost;
//                        ht.traffic_open.decrease_key(existing);
//                    }
//                }
//                else
//                {
//                    if (cost < ht.traffic_htable[next])
//                    {
//                        std::cout << "error in astar: re-expansion" << std::endl;
//                        assert(false);
//                        exit(1);
//                    }
//                }
//            }
//
//        }
//
//        if (source == curr_location)
//            return curr_g;
//
//    }
//
//
//    return MAX_TIMESTEP;
//}


int get_h(SharedEnvironment* env, int source, int target){
	if (global_heuristictable.empty()){
		init_heuristics(env);
	}

	if (global_heuristictable.at(target).empty()){
		init_heuristic(global_heuristictable.at(target),env,target);
	}

	return get_heuristic(global_heuristictable.at(target), env, source, &global_neighbors);
}



void init_dist_2_path(Dist2Path& dp, SharedEnvironment* env, Traj& path){
	if (dp.dist2path.empty())
		dp.dist2path.resize(env->map.size(), d2p(0,-1,MAX_TIMESTEP,MAX_TIMESTEP));
	
	dp.open.clear();
	dp.label++;

    int togo = 0;
    for(int i = path.size()-1; i>=0; i--){
        auto p = path[i];
		assert(dp.dist2path[p].label != dp.label || dp.dist2path[p].cost == MAX_TIMESTEP);
		dp.open.emplace_back(dp.label,p,0,togo);
		dp.dist2path[p] = {dp.label,p,0,togo};
		togo++;
    }

}

std::pair<int,int> get_source_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns)
{
	if (dp.dist2path[source].label == dp.label && dp.dist2path[source].cost < MAX_TIMESTEP){
		// std::cout<<dp.dist2path[source].first<<" "<<dp.dist2path[source].second<<std::endl;

		return std::make_pair(dp.dist2path[source].cost, dp.dist2path[source].togo);
	}

	
	std::vector<int> neighbors;
	int cost;

	while (!dp.open.empty())
	{
		d2p curr = dp.open.front();
		dp.open.pop_front();



		getNeighborLocs(ns,neighbors,curr.id);

		for (int next_location : neighbors)
		{

			cost = curr.cost + 1;

			if (dp.dist2path[next_location].label == dp.label && cost >= dp.dist2path[next_location].cost )
				continue;
			dp.open.emplace_back(dp.label,next_location,cost,curr.togo);
			dp.dist2path[next_location] = {dp.label,next_location,cost,curr.togo};
			
		}
		if (source == curr.id){
			// std::cout<<curr.second.first<<" "<<curr.second.second<<std::endl;
			return std::make_pair(curr.cost, curr.togo);
		}
	}

	return std::make_pair(MAX_TIMESTEP,0);
}

int get_dist_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns)
{

	std::pair<int, int> dists = get_source_2_path(dp,env, source, ns);

	return dists.first + dists.second;
}



}
