


#include "search.h"


namespace DefaultPlanner {
    std::chrono::nanoseconds t;
//a astar minimized the opposide traffic flow with existing traffic flow



    s_node astar(SharedEnvironment *env, std::vector<Float4> &flow,
                 HeuristicTable &ht, MDD_Traj &traj,
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
            traj[start] = {0, {}};
            return *root;
        }

        pqueue_min_of open;

//        re_of re;
        re_of_float re;

        same_of_float same;

        small_than_fmin small_fmin;

        same_with_fmin same_fmin;



        open.push(root);

        int diff, d, cost, depth, p_diff, p_d;
        float op_flow, all_vertex_flow, temp_op, temp_vertex;
        double tie_breaker;

        // variable for MDD paths
        float f_min;
        bool goal_found_flag = false;



        s_node *goal_node = nullptr;
        int neighbors[4];
        int next_neighbors[4];


        while (open.size() > 0) {
            s_node *curr = open.pop();
            curr->close();

            if (curr->id == goal && !goal_found_flag) {
                goal_node = curr;
                goal_found_flag = true;
                f_min = goal_node->get_f() + goal_node->get_op_flow() + goal_node->get_all_vertex_flow();
                continue;  // only close goal_node without expanding it
            }

            // condition of breaking
            if (goal_found_flag)
            {
                if (curr->get_f() + curr->get_op_flow() + curr->get_all_vertex_flow() > f_min)
                    break;
                if (curr->get_f() + curr->get_op_flow() + curr->get_all_vertex_flow() < f_min)
                {
                    std::cout << "smaller cost error" << std::endl;
                    assert(false);
                    exit(1);
                }
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

                p_diff = 0;
                if (curr->parent != nullptr) {
                    p_diff = curr->id - curr->parent->id;
                }

                op_flow += curr->op_flow; //op_flow is contra flow
                all_vertex_flow += curr->all_vertex_flow;

                s_node temp_node(next, cost, h, op_flow, depth);
                temp_node.tie_breaker = tie_breaker;
                temp_node.set_all_flow(op_flow, all_vertex_flow);

                if (!goal_found_flag)
                {
                    if (!mem.has_node(next)) {
                        s_node *next_node = mem.generate_node(next, cost, h, op_flow, depth, all_vertex_flow);
                        next_node->parents[curr->id] = curr;
                        next_node->tie_breaker = tie_breaker;
                        open.push(next_node);
                        generated++;
                    }
                    else {
                        s_node *existing = mem.get_node(next);

                        // A parent exists in the existing.parents indicates expanding a closed node
                        if (existing->parents.find(curr->id) != existing->parents.end())
                        {
                            std::cout << "error in astar: re-expansion" << std::endl;
                            assert(false);
                            exit(1);
                        }

                        if (!existing->is_closed()) {
                            // better cost
                            if (re(temp_node, *existing)) {
                                existing->g = cost;
                                existing->parents.clear();
                                existing->parents[curr->id] = curr;
                                existing->depth = depth;
                                existing->tie_breaker = tie_breaker;
                                existing->set_all_flow(op_flow, all_vertex_flow);
                                open.decrease_key(existing);
                            }
                            // the same cost but a different parent
                            else if (same(temp_node, *existing))
                                existing->parents[curr->id] = curr;
                        }
                        else {
                            if (re(temp_node, *existing)) {
                                std::cout << "error in astar: re-expansion" << std::endl;
                                assert(false);
                                exit(1);
                            }
                            // the same cost but a different parent
                            else if (same(temp_node, *existing))
                                existing->parents[curr->id] = curr;
                        }
                    }
                }
                else
                {
                    if (small_fmin(temp_node, f_min))
                    {
                        std::cout << "error" << std::endl;
                        assert(false);
                        exit(1);
                    }

                    if (same_fmin(temp_node, f_min))
                    {
                        if (!mem.has_node(next)) {
                            s_node *next_node = mem.generate_node(next, cost, h, op_flow, depth, all_vertex_flow);
                            next_node->parents[curr->id] = curr;
                            next_node->tie_breaker = tie_breaker;
                            open.push(next_node);
                            generated++;
                        }
                        else {
                            s_node *existing = mem.get_node(next);

                            // A parent exists in the existing.parents indicates expanding a closed node
                            if (existing->parents.find(curr->id) != existing->parents.end())
                            {
                                std::cout << "error in astar: re-expansion" << std::endl;
                                assert(false);
                                exit(1);
                            }

                            if (!existing->is_closed()) {
                                // better cost
                                if (re(temp_node, *existing)) {
                                    existing->g = cost;
                                    existing->parents.clear();
                                    existing->parents[curr->id] = curr;
                                    existing->depth = depth;
                                    existing->tie_breaker = tie_breaker;
                                    existing->set_all_flow(op_flow, all_vertex_flow);
                                    open.decrease_key(existing);
                                }
                                // the same cost but a different parent
                                else if (same(temp_node, *existing))
                                    existing->parents[curr->id] = curr;
                            }
                            else {
                                if (re(temp_node, *existing)) {
                                    std::cout << "error in astar: re-expansion" << std::endl;
                                    assert(false);
                                    exit(1);
                                }
                                // the same cost but a different parent
                                else if (same(temp_node, *existing))
                                    existing->parents[curr->id] = curr;
                            }

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


        // define the weight in goal
        float weight = 1;
        float curr_weight, next_weight;
        int next_loc, prev_loc, loc_diff, loc_d;

        // record the connection in traj and add_traj
        std::queue<pair<s_node*, float>> nodes;
        nodes.emplace(goal_node, weight);
        traj[goal_node->id] = {weight, {}};


        while (!nodes.empty())
        {
            s_node* curr_node = nodes.front().first;
            curr_weight = nodes.front().second;
            nodes.pop();
            if (curr_node->id == start) continue;


            next_weight = curr_weight / curr_node->parents.size();

            if (curr_node->parents.size() != 0)
            {
                int xxx = 123;
            }

            next_loc = curr_node->id;

            // check existence of curr_node's parents in traj
            bool add_flag = false;
            if (traj[curr_node->id].second.empty())
                add_flag = true;


            for (auto next_node : curr_node->parents)
            {
                if (add_flag)
                    traj[curr_node->id].second.push_back(next_node.first);

                nodes.emplace(next_node.second, next_weight);
                prev_loc = next_node.first;
                loc_diff = next_loc - prev_loc;
                loc_d = get_d(loc_diff, env);
                flow[prev_loc].d[loc_d] += next_weight;

                if (traj.find(next_node.first) == traj.end())
                    traj[next_node.first] = {next_weight, {}};
                else
                    traj[next_node.first].first = traj[next_node.first].first + next_weight;
            }
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
////                    temp_op = std::pow(flow[next].d[(d + 2) % 4], (flow[curr->id].d[d] + 1));
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
