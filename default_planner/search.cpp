


#include "search.h"


namespace DefaultPlanner {
    std::chrono::nanoseconds t;
//a astar minimized the opposide traffic flow with existing traffic flow

    int simulation_astar(SharedEnvironment *env,
                         std::vector<std::vector<std::pair<bool, std::array<bool, 4>>>> &constraint_flow,
                         std::vector<Int4> &flow,
                         HeuristicTable &ht, HeuristicTable &temp_ht, Traj &traj,
                         MemoryPool &mem, int start, int temp_goal, int goal, int start_g, Neighbors *ns) {
        mem.reset();

        int expanded = 0;
        int generated = 0;
        int standard_h = 0;

        if (ht.empty())
            standard_h = manhattanDistance(start, goal, env);
        else
            standard_h = get_heuristic(ht, env, start, ns);


        s_node *root = new s_node(start, env->curr_timestep, start_g, standard_h, 0, 0);


        pqueue_min_of open;
        open.push(root);


        int diff, d, rev_d, cost, op_flow, total_cross, all_vertex_flow, vertex_flow, depth, p_diff, p_d;
        int next_d1, next_d2, next_d1_loc, next_d2_loc;
        int temp_op, temp_vertex;
        double tie_breaker, decay_factor;
        bool traverse_flag = false;

        s_node *goal_node = nullptr;
        int neighbors[4];
        int next_neighbors[4];



        // use A* to evaluate delays
        while (open.size() > 0) {
            s_node *curr = open.pop();

            if (curr->id == temp_goal){
                traverse_flag = true;
            }

            if (traverse_flag)
            {
                if (goal == temp_goal)
                {
                    if (curr->id == goal)
                        return curr->get_g() - (start_g + 1);
                }
                else
                {
                    if (get_heuristic(ht, env, curr->id, ns) < get_heuristic(ht, env, temp_goal, ns))
                        return curr->get_g() - (start_g + 2);
                }
            }


            getNeighborLocs(ns, neighbors, curr->id);
            int next;
            std::vector<int> available_set;
            std::vector<int> tile_set;

            for (int i = 0; i < 5; i++) {
                if (i == 4)
                    next = curr->id;
                else
                    next = neighbors[i];

                if (next == -1)
                    continue;

                // find out which edge and reverse edge
                diff = next - curr->id;
                d = get_d(diff, env);
                rev_d = (d < 2) ? (d + 2) : (d - 2);
                tile_set.push_back(next);


                // dynamic obstacle
                if (constraint_flow[curr->curr_timestep + curr->get_g() + 1][next].first ||  // vertex conflict
                    constraint_flow[curr->curr_timestep + curr->get_g()][next].second[rev_d])  // edge conflict
                    continue;

                available_set.push_back(next);
            }


            if (available_set.empty())
                return -1;


            if (traverse_flag)
            {
                std::sort(available_set.begin(), available_set.end(), [&](int a, int b) {
                    return get_heuristic(ht, env, a, ns) < get_heuristic(ht, env, b, ns);
                });
            }
            else
            {
                std::sort(available_set.begin(), available_set.end(), [&](int a, int b) {
                    return get_heuristic(temp_ht, env, a, ns) < get_heuristic(temp_ht, env, b, ns);
                });
            }

            int next_id = available_set[0];
            int next_h = 0;

            if (traverse_flag)
                next_h = get_heuristic(ht, env, next_id, ns);
            else
                next_h = get_heuristic(temp_ht, env, next_id, ns);


            // find the valid child
            s_node* next_node = new s_node(next_id, curr->g + 1, next_h,0, curr->depth + 1);
            next_node->parent = curr;
            next_node->tie_breaker = tie_breaker;
            next_node->all_vertex_flow = 0;
            open.push(next_node);

        }

    }


    s_node astar(SharedEnvironment* env, std::vector<std::vector<std::pair<bool, std::array<bool, 4>>>>& constraint_flow,
                 std::vector<Int4>& flow,
                 HeuristicTable& ht, std::vector<HeuristicTable>& all_ht, Traj& traj,
                 MemoryPool& mem, int start, int goal, Neighbors* ns)
    {
        mem.reset();

        int expanded=0;
        int generated=0;
        int h;

        if(ht.empty())
            h = manhattanDistance(start,goal,env);
        else
            h = get_heuristic(ht,env, start, ns);


        s_node* root = new s_node(start, env->curr_timestep, 0, h, 0, 0);

        if (start == goal){
            traj.clear();
            traj.push_back(start);
            return *root;
        }

        pqueue_min_of open;
        re_of re;

        std::unordered_map<int, s_node*> node_check_list;


        open.push(root);
        node_check_list[root->id] = root;


        int  diff, d, rev_d, cost, op_flow, total_cross, all_vertex_flow,vertex_flow, depth,p_diff, p_d;
        int next_d1, next_d2, next_d1_loc, next_d2_loc;
        int temp_op, temp_vertex;
        double tie_breaker, decay_factor;

        s_node* goal_node = nullptr;
        int neighbors[4];
        int next_neighbors[4];



        while (open.size() > 0){
            s_node* curr = open.pop();
            curr->close();

            if (curr->id == goal){
                goal_node = curr;
                break;
            }

            getNeighborLocs(ns,neighbors,curr->id);
            int next;


            for (int i=0; i<4; i++){
                next = neighbors[i];

                if (next == -1){
                    continue;
                }

                op_flow = 0;
                all_vertex_flow = 0;

                cost = curr->g+1;

                assert(next >= 0 && next < env->map.size());
                depth = curr->depth + 1;


                if(ht.empty())
                    h = manhattanDistance(next,goal,env);
                else
                    h = get_heuristic(ht,env, next, ns);


                diff = next - curr->id;
                d = get_d(diff,env);
                rev_d = (d < 2) ? (d + 2) : (d - 2);


                if (curr->parent != nullptr){
                    p_diff = curr->id - curr->parent->id;
                    p_d = get_d(p_diff,env);
                    if (p_d!=d)
                        tie_breaker = 0.1;
                    else
                        tie_breaker = 0;
                    //tie breaking on prefering moving forward
                }

                temp_op = ((flow[curr->id].d[d] + 1) *
                           flow[next].d[(d + 2) % 4]);///( ( (flow[curr->id].d[d]+1) + flow[next].d[(d+2)%4]));


                //all vertex flow
                //the sum of all out going edge flow is the same as the total number of vertex visiting.
                temp_vertex = 1;
                for (int j = 0; j < 4; j++) {
                    temp_vertex += flow[next].d[j];
                }

                op_flow += temp_op;

                all_vertex_flow += (temp_vertex - 1) / 2;


                // dynamic obstacle
                if (constraint_flow[curr->curr_timestep + curr->get_g() + 1][next].first ||  // vertex conflict
                    constraint_flow[curr->curr_timestep + curr->get_g()][next].second[rev_d])  // edge conflict
                {
                    if (all_ht.at(next).empty()){
                        init_heuristic(all_ht[next],env,next);
                    }
                    int op_result = simulation_astar(env, constraint_flow, flow, ht, all_ht[next],
                                                     traj,mem, curr->id, next,
                                                     goal, curr->g, ns);
                    if (op_result != -1)
                        op_flow = op_flow + op_result;
//                        op_flow = std::max(op_flow, op_result);
                    else
                    {
                        op_flow = op_flow + INT_MAX / 10;
                    }
                }


                p_diff = 0;
                if (curr->parent != nullptr){
                    p_diff = curr->id - curr->parent->id;
                }

                op_flow += curr->op_flow; //op_flow is contra flow

                all_vertex_flow += curr->all_vertex_flow;



                if (node_check_list.find(next) == node_check_list.end())
                {
                    s_node* next_node = new s_node(next, env->curr_timestep, cost, h, op_flow, depth);
                    next_node->parent = curr;
                    next_node->tie_breaker = tie_breaker;
                    next_node->all_vertex_flow = all_vertex_flow;
                    open.push(next_node);
                    node_check_list[next_node->id] = next_node;
                }
                else
                {
                    auto temp_node = node_check_list[next];
                    if (!temp_node->is_closed())
                    {
                        int curr_cost = temp_node->get_f() + temp_node->get_op_flow() + temp_node->get_all_vertex_flow();
                        if (cost + h + op_flow + all_vertex_flow < curr_cost)
                        {
                            node_check_list[next]->g = cost;
                            node_check_list[next]->parent = curr;
                            node_check_list[next]->tie_breaker = tie_breaker;
                            node_check_list[next]->op_flow = op_flow;
                            node_check_list[next]->all_vertex_flow = all_vertex_flow;
                            node_check_list[next]->depth = depth;
                            open.decrease_key(node_check_list[next]);
                        }
                    }

                }

            }

        }


        traj.resize(goal_node->depth+1);
        s_node* curr = goal_node;
        for (int i=goal_node->depth; i>=0; i--){
            traj[i] = curr->id;
            curr = curr->parent;
        }

        return *goal_node;
    }



    // original
    s_node astar(SharedEnvironment *env, std::vector<std::vector<Int4>>& flow_with_time,
                 HeuristicTable &ht, Traj &traj,
                 MemoryPool &mem, int start, int goal, Neighbors *ns) {
        mem.reset();

        int expanded = 0;
        int generated = 0;
        int h;

        if (ht.empty())
            h = manhattanDistance(start, goal, env);
        else
            h = get_heuristic(ht, env, start, ns);



        s_node *root = mem.generate_node(start, 0, h, 0, 0, 0);

        if (start == goal) {
            traj.clear();
            traj.push_back(start);
            return *root;
        }

        pqueue_min_of open;
        re_of re;

        open.push(root);

        int diff, d, cost, op_flow, total_cross, all_vertex_flow, vertex_flow, depth, p_diff, p_d;
        int next_d1, next_d2, next_d1_loc, next_d2_loc;
        int temp_op, temp_vertex;
        double tie_breaker, decay_factor;

        s_node *goal_node = nullptr;
        int neighbors[4];
        int next_neighbors[4];


        while (open.size() > 0) {
            s_node *curr = open.pop();
            curr->close();

            if (curr->id == goal) {
                goal_node = curr;
                break;
            }
            expanded++;
            getNeighborLocs(ns, neighbors, curr->id);

            for (int i = 0; i < 4; i++) {
                int next = neighbors[i];
                if (next == -1) {
                    continue;
                }

                cost = curr->g + 1;

                assert(next >= 0 && next < env->map.size());
                depth = curr->depth + 1;

                //moving direction
                //flow
                op_flow = 0;
                all_vertex_flow = 0;

                if (ht.empty())
                    h = manhattanDistance(next, goal, env);
                else
                    h = get_heuristic(ht, env, next, ns);

                diff = next - curr->id;
                d = get_d(diff, env);
                if (curr->parent != nullptr) {
                    p_diff = curr->id - curr->parent->id;
                    p_d = get_d(p_diff, env);
                    if (p_d != d)
                        tie_breaker = 0.1;
                    else
                        tie_breaker = 0;
                    //tie breaking on prefering moving forward
                }


                temp_op = ((flow_with_time[curr->curr_timestep + curr->get_g()][curr->id].d[d] + 1) *
                           flow_with_time[curr->curr_timestep + curr->get_g()][next].d[(d + 2) % 4]);///( ( (flow[curr->id].d[d]+1) + flow[next].d[(d+2)%4]));

                //all vertex flow
                //the sum of all out going edge flow is the same as the total number of vertex visiting.
                temp_vertex = 1;
                for (int j = 0; j < 4; j++) {
                    temp_vertex += flow_with_time[curr->curr_timestep + curr->get_g()][next].d[j];
                }

                op_flow += temp_op;

                all_vertex_flow += (temp_vertex - 1) / 2;

                p_diff = 0;
                if (curr->parent != nullptr) {
                    p_diff = curr->id - curr->parent->id;
                }

                op_flow += curr->op_flow; //op_flow is contra flow
                all_vertex_flow += curr->all_vertex_flow;

                s_node temp_node(next, cost, h, op_flow, depth);
                temp_node.tie_breaker = tie_breaker;
                temp_node.set_all_flow(op_flow, all_vertex_flow);

                if (!mem.has_node(next)) {
                    s_node *next_node = mem.generate_node(next, cost, h, op_flow, depth, all_vertex_flow);
                    next_node->parent = curr;
                    next_node->tie_breaker = tie_breaker;
                    open.push(next_node);
                    generated++;
                } else {
                    s_node *existing = mem.get_node(next);

                    if (!existing->is_closed()) {
                        if (re(temp_node, *existing)) {
                            existing->g = cost;
                            existing->parent = curr;
                            existing->depth = depth;
                            existing->tie_breaker = tie_breaker;
                            existing->set_all_flow(op_flow, all_vertex_flow);
                            open.decrease_key(existing);
                        }
                    } else {

                        if (re(temp_node, *existing)) {
                            std::cout << "error in astar: re-expansion" << std::endl;
                            assert(false);
                            exit(1);
                        }

                    }
                }
            }


        }


        if (goal_node == nullptr) {
            std::cout << "error in astar: no path found " << start << "," << goal << std::endl;
            assert(false);
            exit(1);
        }

        traj.resize(goal_node->depth + 1);
        s_node *curr = goal_node;
        for (int i = goal_node->depth; i >= 0; i--) {
            traj[i] = curr->id;
            curr = curr->parent;
        }

        return *goal_node;
    }




//        // original
//        s_node astar(SharedEnvironment *env, std::vector<Int4> &flow,
//                     HeuristicTable &ht, Traj &traj,
//                     MemoryPool &mem, int start, int goal, Neighbors *ns) {
//            mem.reset();
//
//            int expanded = 0;
//            int generated = 0;
//            int h;
//
//            if (ht.empty())
//                h = manhattanDistance(start, goal, env);
//            else
//                h = get_heuristic(ht, env, start, ns);
//
//
//
//            s_node *root = mem.generate_node(start, 0, h, 0, 0, 0);
//
//            if (start == goal) {
//                traj.clear();
//                traj.push_back(start);
//                return *root;
//            }
//
//            pqueue_min_of open;
//            re_of re;
//
//            open.push(root);
//
//            int diff, d, cost, op_flow, total_cross, all_vertex_flow, vertex_flow, depth, p_diff, p_d;
//            int next_d1, next_d2, next_d1_loc, next_d2_loc;
//            int temp_op, temp_vertex;
//            double tie_breaker, decay_factor;
//
//            s_node *goal_node = nullptr;
//            int neighbors[4];
//            int next_neighbors[4];
//
//
//            while (open.size() > 0) {
//                s_node *curr = open.pop();
//                curr->close();
//
//                if (curr->id == goal) {
//                    goal_node = curr;
//                    break;
//                }
//                expanded++;
//                getNeighborLocs(ns, neighbors, curr->id);
//
//                for (int i = 0; i < 4; i++) {
//                    int next = neighbors[i];
//                    if (next == -1) {
//                        continue;
//                    }
//
//                    cost = curr->g + 1;
//
//                    assert(next >= 0 && next < env->map.size());
//                    depth = curr->depth + 1;
//
//                    //moving direction
//                    //flow
//                    op_flow = 0;
//                    all_vertex_flow = 0;
//
//                    if (ht.empty())
//                        h = manhattanDistance(next, goal, env);
//                    else
//                        h = get_heuristic(ht, env, next, ns);
//
//                    diff = next - curr->id;
//                    d = get_d(diff, env);
//                    if (curr->parent != nullptr) {
//                        p_diff = curr->id - curr->parent->id;
//                        p_d = get_d(p_diff, env);
//                        if (p_d != d)
//                            tie_breaker = 0.1;
//                        else
//                            tie_breaker = 0;
//                        //tie breaking on prefering moving forward
//                    }
//
//
//                    temp_op = ((flow[curr->id].d[d] + 1) *
//                               flow[next].d[(d + 2) % 4]);///( ( (flow[curr->id].d[d]+1) + flow[next].d[(d+2)%4]));
//
//                    //all vertex flow
//                    //the sum of all out going edge flow is the same as the total number of vertex visiting.
//                    temp_vertex = 1;
//                    for (int j = 0; j < 4; j++) {
//                        temp_vertex += flow[next].d[j];
//                    }
//
//                    op_flow += temp_op;
//
//                    all_vertex_flow += (temp_vertex - 1) / 2;
//
//                    p_diff = 0;
//                    if (curr->parent != nullptr) {
//                        p_diff = curr->id - curr->parent->id;
//                    }
//
//                    op_flow += curr->op_flow; //op_flow is contra flow
//                    all_vertex_flow += curr->all_vertex_flow;
//
//                    s_node temp_node(next, cost, h, op_flow, depth);
//                    temp_node.tie_breaker = tie_breaker;
//                    temp_node.set_all_flow(op_flow, all_vertex_flow);
//
//                    if (!mem.has_node(next)) {
//                        s_node *next_node = mem.generate_node(next, cost, h, op_flow, depth, all_vertex_flow);
//                        next_node->parent = curr;
//                        next_node->tie_breaker = tie_breaker;
//                        open.push(next_node);
//                        generated++;
//                    } else {
//                        s_node *existing = mem.get_node(next);
//
//                        if (!existing->is_closed()) {
//                            if (re(temp_node, *existing)) {
//                                existing->g = cost;
//                                existing->parent = curr;
//                                existing->depth = depth;
//                                existing->tie_breaker = tie_breaker;
//                                existing->set_all_flow(op_flow, all_vertex_flow);
//                                open.decrease_key(existing);
//                            }
//                        } else {
//
//                            if (re(temp_node, *existing)) {
//                                std::cout << "error in astar: re-expansion" << std::endl;
//                                assert(false);
//                                exit(1);
//                            }
//
//                        }
//                    }
//                }
//
//
//            }
//
//
//            if (goal_node == nullptr) {
//                std::cout << "error in astar: no path found " << start << "," << goal << std::endl;
//                assert(false);
//                exit(1);
//            }
//
//            traj.resize(goal_node->depth + 1);
//            s_node *curr = goal_node;
//            for (int i = goal_node->depth; i >= 0; i--) {
//                traj[i] = curr->id;
//                curr = curr->parent;
//            }
//
//            return *goal_node;
//        }


    }
