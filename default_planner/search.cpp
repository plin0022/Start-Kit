


#include "search.h"


namespace DefaultPlanner {
    std::chrono::nanoseconds t;
//a astar minimized the opposide traffic flow with existing traffic flow



    s_node astar(SharedEnvironment *env, std::vector<Float4> &flow,
                 HeuristicTable &ht, MDD_Traj &traj,
                 MemoryPool &mem, int start, int goal, Neighbors *ns) {
        mem.reset();


        int h;

        if (ht.empty())
            h = manhattanDistance(start, goal, env);
        else
            h = get_heuristic(ht, env, start, ns);


        s_node *root = mem.generate_node(start, 0, h, 0, 0, 0);


        if (start == goal) {
            traj.clear();
            traj[start] = {};
            return *root;
        }

        pqueue_min_of open;
        re_of re;

        open.push(root);

        // variable for MDD paths
        int f_min, f_break;
        bool goal_found_flag = false;


        int diff, d, cost, op_flow, all_vertex_flow, vertex_flow, depth;
        int temp_op, temp_vertex;
        double tie_breaker;

        s_node *goal_node = nullptr;
        int neighbors[4];
        int next_neighbors[4];


        while (open.size() > 0) {
            s_node *curr = open.pop();
            curr->close();

            // only go into this 'if' once
            if (curr->id == goal && !goal_found_flag) {
                goal_node = curr;
                goal_found_flag = true;
                f_min = goal_node->get_all_costs();
                f_break = 1 * f_min;
                continue;  // only close goal_node without expanding it
            }

            // condition of breaking
            if (goal_found_flag)
            {
                if (curr->get_all_costs() > f_break)
                    break;
                if (curr->get_all_costs() < f_break)
                {
                    std::cout << "smaller cost error" << std::endl;
                    assert(false);
                    exit(1);
                }
            }

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
                            else if (temp_node.get_all_costs() == existing->get_all_costs())
                                existing->parents[curr->id] = curr;
                        }
                        else {
                            if (re(temp_node, *existing)) {
                                std::cout << "error in astar: re-expansion" << std::endl;
                                assert(false);
                                exit(1);
                            }
                            // the same cost but a different parent
                            else if (temp_node.get_all_costs() == existing->get_all_costs())
                                existing->parents[curr->id] = curr;
                        }
                    }
                }
                else
                {
                    if (temp_node.get_all_costs() < f_break)
                    {
                        std::cout << "temp_node small costs error" << std::endl;
                        assert(false);
                        exit(1);
                    }

                    if (temp_node.get_all_costs() == f_break)
                    {
                        if (!mem.has_node(next)) {
                            s_node *next_node = mem.generate_node(next, cost, h, op_flow, depth, all_vertex_flow);
                            next_node->parents[curr->id] = curr;
                            next_node->tie_breaker = tie_breaker;
                            open.push(next_node);
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
                                else if (temp_node.get_all_costs() == existing->get_all_costs())
                                    existing->parents[curr->id] = curr;
                            }
                            else {
                                if (re(temp_node, *existing)) {
                                    std::cout << "error in astar: re-expansion" << std::endl;
                                    assert(false);
                                    exit(1);
                                }
                                // the same cost but a different parent
                                else if (temp_node.get_all_costs() == existing->get_all_costs())
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



        // backward search from goal to find out all the parents-children relationships
        std::unordered_map<int, std::unordered_map<int, int>> parent_children;  // first int is parent_id, inside map is children
        std::vector<bool> visited(env->map.size(), false);  // record if a node is visited before
        std::queue<s_node*> nodes;

        nodes.emplace(goal_node);
        visited[goal_node->id] = true;

        // build parent_children connections
        while (!nodes.empty())
        {
            s_node* curr_node = nodes.front();
            nodes.pop();

            for (auto parent_node : curr_node->parents)
            {
                if (!visited[parent_node.first])
                {
                    nodes.emplace(parent_node.second);
                    visited[parent_node.first] = true;
                }
                parent_children[parent_node.first][curr_node->id] = curr_node->id;
            }
        }

        // weight table
        std::vector<float> id_weight(env->map.size(), 0);
        id_weight[start] = 1;

        // open and closed
        std::vector<bool> id_open(env->map.size(),false);
        std::vector<bool> id_closed(env->map.size(),false);

        // proceed the node in a task finished manner
        std::deque<int> id_que;
        id_que.push_front(start);
        id_open[start] = true;
        std::vector<int> ready_map(env->map.size(),5);
        for (auto each : parent_children)
        {
            ready_map[each.first] = mem.get_node(each.first)->parents.size();
        }
        ready_map[goal] = mem.get_node(goal)->parents.size();  // goal is not in the parent_children


        while (!id_que.empty())
        {
            int curr_id = id_que.front();
            id_que.pop_front();


            if (ready_map[curr_id] == 0)
            {
                // print out the weight for review
//                std::cout << "node:" << curr_id << " " << id_weight[curr_id] << std::endl;


                id_open[curr_id] = false;  // remove it from open
                id_closed[curr_id] = true;  // mark curr_id as expanded
                float curr_weight = 0;

                // setting weight for propagation
                if (parent_children[curr_id].size() != 0)
                {
                    curr_weight = id_weight[curr_id] / parent_children[curr_id].size();
                }

                for (auto child : parent_children[curr_id])
                {
                    if (id_closed[child.first])  // check if a closed node is added into id_que as a child
                    {
                        std::cout << "error: re-add an expanded node" << std::endl;
                        assert(false);
                    }
                    else if (!id_closed[child.first] && id_open[child.first])  // not closed but still in open
                    {
                        ready_map[child.first] = ready_map[child.first] - 1;
                        id_weight[child.first] = id_weight[child.first] + curr_weight;
                    }
                    else if (!id_closed[child.first] && !id_open[child.first])  // first time visit this child
                    {
                        ready_map[child.first] = ready_map[child.first] - 1;

                        if (id_weight[child.first] == 0)
                            id_weight[child.first] = curr_weight;
                        else
                            assert(false);

                        if (ready_map[child.first] == 0)
                            id_que.push_front(child.first);
                        else
                            id_que.push_back(child.first);
                        id_open[child.first] = true;
                    }

                    // update the weight since the curr_id (parent) is ready
                    traj[curr_id][child.first] = curr_weight;

                    // update the flow table
                    int loc = child.first;
                    int prev_loc = curr_id;
                    diff = loc - prev_loc;
                    d = get_d(diff, env);

                    flow[prev_loc].d[d] += curr_weight;
                }
            }
            else if (ready_map[curr_id] < 0)
                assert(false);
            else
                id_que.push_back(curr_id);  // not ready, push it to the tail of the que

        }








        return *goal_node;
    }





}
