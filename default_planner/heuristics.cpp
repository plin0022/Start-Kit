
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
	HNode root(goal_location,0, 0);
	ht.htable[goal_location] = 0;
	ht.open.push_back(root);  // add root to open
}

//void init_flextable(HeuristicTable& ht, SharedEnvironment* env, int goal_location)
//{
//    // initialize flex table
//    ht.flex_table.clear();
//    ht.flex_table.resize(env->map.size(), std::numeric_limits<unsigned int>::max());
//    ht.flex_table[goal_location] = 0;
//}


//uint get_flex(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns)
//{
//    if (ht.flex_table[source] < std::numeric_limits<unsigned int>::max())
//        return ht.flex_table[source];
//
//
//    const uint curr_dis = get_heuristic(ht, env, source, ns);
//    int award_points = 2;
//    uint final_points = 0;
//    std::vector<int> neighbors;
//    getNeighborLocs(ns,neighbors,source);
//
//
//    for (int next : neighbors)
//    {
//        if (get_heuristic(ht, env, next, ns) < curr_dis)
//            final_points = final_points + award_points +
//                    get_flex(ht, env, next, ns);
//    }
//
//    ht.flex_table[source] = final_points;
//
//    return final_points;
//}


//uint get_flex(TrajLNS& lns, int ai, int source, Neighbors* ns)
//{
//    if (lns.heuristics[lns.tasks.at(ai)].flex_table[source] < std::numeric_limits<unsigned int>::max())
//        return lns.heuristics[lns.tasks.at(ai)].flex_table[source];
//
//
//    const int curr_dis = get_gp_h(lns, ai, source);
//    int award_points = 2;
//    uint final_points = 0;
//    std::vector<int> neighbors;
//    getNeighborLocs(ns,neighbors,source);
//
//
//    for (int next : neighbors)
//    {
//        if (get_gp_h(lns, ai, next) < curr_dis)
//            final_points = final_points + award_points +
//                           get_flex(lns, ai,next, ns);
//    }
//
//    lns.heuristics[lns.tasks.at(ai)].flex_table[source] = final_points;
//
//    return final_points;
//}



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
				cost = curr.value + 1;
				diff = curr.location - next;
				
				assert(next >= 0 && next < env->map.size());
				//set current cost for reversed direction

				if (cost >= ht.htable[next] )
					continue;

				ht.open.emplace_back(next,0, cost);
				ht.htable[next] = cost;
				
			}

			if (source == curr.location)
				return curr.value;
		}


		return MAX_TIMESTEP;
}



// traffic heuristic

void init_traffic_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location){
    // initialize my_heuristic, but have error on malloc: Region cookie corrupted for region
    ht.traffic_htable.clear();
    ht.traffic_htable.resize(env->map.size(),MAX_TIMESTEP);


    if (!ht.traffic_open.empty())
    {
        if (ht.traffic_open.size() != ht.node_list.size())
        {
            std::cout << "open not correct" << std::endl;
            assert(false);
            exit(1);
        }
    }

    // free memory of open
    while (!ht.traffic_open.empty())
    {
        HNode *curr = ht.traffic_open.pop();
        delete curr;
    }
    ht.traffic_open.clear();


    ht.node_list.clear();

    // initialize closed
    ht.traffic_closed.clear();
    ht.traffic_closed.resize(env->map.size(),false);



    // generate an open that can save nodes
    HNode* root = new HNode(goal_location, 0, 0);  // dynamically allocate HNode
    ht.traffic_htable[goal_location] = 0;
    ht.traffic_open.push(root);  // add root to open
    ht.node_list[goal_location] = root;
}



int get_traffic_heuristic(TrajLNS& lns, HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns)
{
    if (ht.traffic_closed[source]) return ht.traffic_htable[source];

    std::vector<int> neighbors;
    int cost, diff, d, temp_op, temp_vertex, curr_location, curr_value;

    while (!ht.traffic_open.empty())
    {
        // free memory and remove the entry in map for a closed node
        HNode *curr = ht.traffic_open.pop();
        curr_location = curr->location;
        curr_value = curr->value;
        ht.traffic_htable[curr_location] = curr_value;
        ht.traffic_closed[curr_location] = true;
        delete curr;
        ht.node_list.erase(curr_location);


        getNeighborLocs(ns,neighbors, curr_location);


        for (int next : neighbors)
        {
            diff = curr_location - next;
            d = get_d(diff, env);

            temp_op = ((lns.flow[next].d[d] + 1) *
                       lns.flow[curr_location].d[(d + 2) % 4]);///( ( (flow[curr->id].d[d]+1) + flow[next].d[(d+2)%4]));
            //all vertex flow
            //the sum of all out going edge flow is the same as the total number of vertex visiting.
            temp_vertex = 1;
            for (int j = 0; j < 4; j++) {
                temp_vertex += lns.flow[curr_location].d[j];
            }


//            cost = curr_value + 1 + temp_op + (temp_vertex - 1) / 2;
            cost = curr_value + 1;

            assert(next >= 0 && next < env->map.size());
            //set current cost for reversed direction



            if (ht.traffic_htable[next] == MAX_TIMESTEP)
            {
                HNode* next_node = new HNode(next, 0, cost);
                ht.traffic_open.push(next_node);
                ht.traffic_htable[next] = cost;
                ht.node_list[next] = next_node;
            }

            else
            {
                if (!ht.traffic_closed[next])
                {
                    if (cost < ht.traffic_htable[next])
                    {
                        HNode* existing = ht.node_list[next];
                        existing->value = cost;
                        ht.traffic_htable[next] = cost;
                        ht.traffic_open.decrease_key(existing);
                    }
                }
                else
                {
                    if (cost < ht.traffic_htable[next])
                    {
                        std::cout << "error in astar: re-expansion" << std::endl;
                        assert(false);
                        exit(1);
                    }
                }
            }

        }

        if (source == curr_location)
            return curr_value;

    }


    return MAX_TIMESTEP;
}

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
