


#include "pibt.h"





namespace DefaultPlanner{


int get_gp_h(TrajLNS& lns, int ai, int target, int curr_loc){
    int min_heuristic;


    if (!lns.heuristics[lns.tasks.at(ai)].traffic_closed.empty())
    {
        min_heuristic = get_traffic_heuristic(lns, lns.heuristics[lns.tasks.at(ai)],
                                              lns.env, target, lns.start_locs[ai], &(lns.neighbors));
    }
    else
        min_heuristic = manhattanDistance(target,lns.tasks.at(ai),lns.env);


//    if (!lns.heuristics[lns.tasks.at(ai)].empty())
//        min_heuristic = get_heuristic(lns.heuristics[lns.tasks.at(ai)], lns.env, target, &(lns.neighbors));
//    else
//        min_heuristic = manhattanDistance(target,lns.tasks.at(ai),lns.env);


//    if (!lns.traj_dists.empty() && !lns.traj_dists[ai].empty())
//        min_heuristic = get_dist_2_path(lns.traj_dists[ai], lns.env, target, &(lns.neighbors));
//    else if (!lns.heuristics[lns.tasks.at(ai)].empty())
//        min_heuristic = get_heuristic(lns.heuristics[lns.tasks.at(ai)], lns.env, target, &(lns.neighbors));
//    else
//        min_heuristic = manhattanDistance(target,lns.tasks.at(ai),lns.env);
    
    return min_heuristic;
}


bool causalPIBT(int curr_id, int higher_id,std::vector<State>& prev_states,
	 std::vector<State>& next_states,
      std::vector<int>& prev_decision, std::vector<int>& decision, 
	  std::vector<bool>& occupied, TrajLNS& lns
	  ){
	// The PIBT works like a causal PIBT when using MAPF-T model. But a normal PIBT when using MAPF model.
	
	assert(next_states[curr_id].location == -1);
    int prev_loc = prev_states[curr_id].location;
	int prev_orientation = prev_states[curr_id].orientation;
	int next[4] = { prev_loc + 1,prev_loc + lns.env->cols, prev_loc - 1, prev_loc - lns.env->cols};
	int orien_next_v = next[prev_orientation];

	assert(prev_loc >= 0 && prev_loc < lns.env->map.size());

	int target = lns.tasks.at(curr_id);

	// for each neighbor of (prev_loc,prev_direction), and a wait copy of current location, generate a successor
	std::vector<int> neighbors;
	std::vector<PIBT_C> successors;
	getNeighborLocs(&(lns.neighbors),neighbors,prev_loc);
	for (auto& neighbor: neighbors){

		assert(validateMove(prev_loc, neighbor, lns.env));

		int min_heuristic = get_gp_h(lns, curr_id, neighbor, prev_loc);


		successors.emplace_back(neighbor,min_heuristic,-1,rand());
	}

	int wait_heuristic = get_gp_h(lns, curr_id, prev_loc, prev_loc);


	successors.emplace_back(prev_loc, wait_heuristic,-1,rand());




	std::sort(successors.begin(), successors.end(), 
		[&](PIBT_C& a, PIBT_C& b)
		{
            // prefer the tile which optimizes the current tile (A* updates with better costs)
            int diff_a = a.location - prev_loc;
            int diff_b = b.location - prev_loc;
            int d_a = get_d(diff_a, lns.env);
            int d_b = get_d(diff_b, lns.env);

            int temp_op_a = (lns.flow[prev_loc].d[d_a] + 1) * lns.flow[a.location].d[(d_a + 2) % 4];
            int temp_op_b = (lns.flow[prev_loc].d[d_b] + 1) * lns.flow[b.location].d[(d_b + 2) % 4];

            int temp_vertex_a = 1;
            for (int j = 0; j < 4; j++) {
                temp_vertex_a += lns.flow[a.location].d[j];
            }

            int temp_vertex_b = 1;
            for (int j = 0; j < 4; j++) {
                temp_vertex_b += lns.flow[b.location].d[j];
            }


            if ((a.heuristic + 1 + temp_op_a + (temp_vertex_a - 1) / 2) == wait_heuristic &&
            (b.heuristic + 1 + temp_op_b + (temp_vertex_b - 1) / 2) != wait_heuristic)
                return true;
            else if ((a.heuristic + 1 + temp_op_a + (temp_vertex_a - 1) / 2) != wait_heuristic &&
                (b.heuristic + 1 + temp_op_b + (temp_vertex_b - 1) / 2) == wait_heuristic)
                return false;
            else
            {
                if (a.heuristic == b.heuristic){

                    // random tie break
                    return a.tie_breaker < b.tie_breaker;
                }
                return a.heuristic < b.heuristic;
            }


//            if (a.heuristic == b.heuristic){
//                //tie break on prefer moving forward
//                if (a.location==orien_next_v && b.location!=orien_next_v)
//                    return true;
//                if (a.location!=orien_next_v && b.location==orien_next_v)
//                    return false;
//                // random tie break
//                return a.tie_breaker < b.tie_breaker;
//            }
//            return a.heuristic < b.heuristic;
        });


    for (auto& next: successors){
		if(occupied[next.location])
			continue;
		assert(validateMove(prev_loc, next.location, lns.env));
		
		if (next.location == -1)
			continue;
		if (decision[next.location] != -1){
			continue;
		}
		if (higher_id != -1 && prev_decision[next.location] == higher_id){
			continue;
		}
		next_states.at(curr_id) = State(next.location, -1, -1);
		decision.at(next.location) = curr_id;

        if (prev_decision.at(next.location) != -1 && 
			next_states.at(prev_decision.at(next.location)).location == -1){
            int lower_id = prev_decision.at(next.location);
            if (!causalPIBT(lower_id,curr_id,prev_states,next_states, prev_decision,decision, occupied,lns)){
				continue;
            }
        }

        return true;
    }

    next_states.at(curr_id) = State(prev_loc,-1 ,-1);;
    decision.at(prev_loc) = curr_id;     

	#ifndef NDEBUG
		std::cout<<"false: "<< next_states[curr_id].location<<","<<next_states[curr_id].orientation <<std::endl;
	#endif   

    return false;
}


Action getAction(State& prev, int next_loc, SharedEnvironment* env){
    if (prev.location == next_loc){
        return Action::W;
    }
    if (next_loc-prev.location ==1)
        return Action::R;
    if (next_loc-prev.location ==-1)
        return Action::L;
    if (next_loc-prev.location == env->cols)
        return Action::D;
    if (next_loc-prev.location == -env->cols)
        return Action::U;

    return Action::W;


}


}