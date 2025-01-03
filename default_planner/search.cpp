


#include "search.h"


namespace DefaultPlanner{
std::chrono::nanoseconds t;
//a astar minimized the opposide traffic flow with existing traffic flow

s_node astar(SharedEnvironment* env, std::vector<Int4>& flow,
    HeuristicTable& ht, Traj& traj,
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
    

    
    s_node* root = mem.generate_node(start,0, h,0,0,0);

    if (start == goal){
        traj.clear();
        traj.push_back(start);
        return *root;
    }

    pqueue_min_of open;
    re_of re;
    std::unordered_map<int, s_node*> closed;
    bool g_found_flag = false;
    int f_min = 0;


    open.push(root);

    int  diff, d, cost, op_flow, total_cross, all_vertex_flow,vertex_flow, depth,p_diff, p_d;
    int next_d1, next_d2, next_d1_loc, next_d2_loc;
    int temp_op, temp_vertex;
    double tie_breaker, decay_factor;

    s_node* goal_node = nullptr;
    int neighbors[4];
    int next_neighbors[4];



    while (open.size() > 0){
        s_node* curr = open.pop();


        if (curr->id == goal && !g_found_flag){
            goal_node = curr;
            g_found_flag = true;
            f_min = curr->get_all_vertex_flow() + curr->get_op_flow() + curr->get_f();
            closed[curr->id] = curr;
            curr->close();
            continue;  // do not expand goal node
        }

        if (g_found_flag)
        {
            int curr_tf = curr->get_all_vertex_flow() + curr->get_op_flow() + curr->get_f();
            if (curr_tf < f_min)
            {
                std::cout << "error in astar: smaller cost error" << std::endl;
                assert(false);
                exit(1);
            }
            if (curr_tf == f_min)
            {
//                std::cout << "another potential node on traffic optimal path found" << std::endl;
            }
            if (curr_tf > f_min)
            {
                break;
            }
        }


        closed[curr->id] = curr;
        curr->close();
        expanded++;
        getNeighborLocs(ns,neighbors,curr->id);

        // build connections between children and the current node
        for (int i=0; i<4; i++){
            int next = neighbors[i];
            if (next == -1){
                continue;
            }

            cost = curr->g+1;

            assert(next >= 0 && next < env->map.size());
            depth = curr->depth + 1;

            //moving direction
            //flow
            op_flow = 0;
            all_vertex_flow = 0;

            if(ht.empty())
                h = manhattanDistance(next,goal,env);
            else
                h = get_heuristic(ht,env, next, ns);

            diff = next - curr->id;
            d = get_d(diff,env);


            temp_op = ( (flow[curr->id].d[d]+1) * flow[next].d[(d+2)%4]);///( ( (flow[curr->id].d[d]+1) + flow[next].d[(d+2)%4]));

            //all vertex flow
            //the sum of all out going edge flow is the same as the total number of vertex visiting.
            temp_vertex = 1;
            for (int j=0; j<4; j++){
                temp_vertex += flow[next].d[j];                
            }

            op_flow += temp_op;
        
            all_vertex_flow+= (temp_vertex-1) /2;


            op_flow += curr->op_flow; //op_flow is contra flow
            all_vertex_flow += curr->all_vertex_flow;

            s_node temp_node(next,cost,h,op_flow, depth);
            temp_node.tie_breaker = tie_breaker;
            temp_node.set_all_flow(op_flow,  all_vertex_flow);
            int traffic_fcost = temp_node.get_op_flow() + temp_node.get_all_vertex_flow() + temp_node.get_f();


            // the node has never been visited before
            if (!mem.has_node(next)){
                s_node* next_node = mem.generate_node(next,cost,h,op_flow, depth,all_vertex_flow);
                next_node->parents[curr->id] = curr;
                next_node->tie_breaker = tie_breaker;
                open.push(next_node);
                generated++;
            }

            // the node has been visited before
            else{
                s_node* existing = mem.get_node(next);

                // the node is in open
                if (!existing->is_closed()){
                    // if the cost is better
                    if (re(temp_node,*existing)){
                        existing->g = cost;
                        existing->parents.clear();
                        existing->parents[curr->id] = curr;
                        existing->depth = depth;
                        existing->tie_breaker = tie_breaker;
                        existing->set_all_flow(op_flow,  all_vertex_flow);
                        open.decrease_key(existing);
                    }

                    // the same cost
                    if ((existing->get_op_flow() + existing->get_all_vertex_flow() + existing->get_f()) ==
                        traffic_fcost)
                    {
                        // new parent
                        if (existing->parents.find(curr->id) == existing->parents.end())
                            existing->parents[curr->id] = curr;
                    }
                }

                // the node is in closed
                else{
                    if (re(temp_node,*existing)){
                        std::cout << "error in astar: re-expansion" << std::endl;
                        assert(false);
                        exit(1);
                    }

                    // the same cost
                    if ((existing->get_op_flow() + existing->get_all_vertex_flow() + existing->get_f()) ==
                        traffic_fcost)
                    {
                        // new parent
                        if (closed[existing->id]->parents.find(curr->id) == closed[existing->id]->parents.end())
                            closed[existing->id]->parents[curr->id] = curr;
                    }
                }
            }
        }
    }


    if (goal_node == nullptr){
        std::cout << "error in astar: no path found "<< start<<","<<goal << std::endl;
        assert(false);
        exit(1);
    }

    // backward to retrieve all the nodes on optimal paths
    std::unordered_map<int, s_node*> optimal_traj;
    std::queue<s_node*> nodes;
    nodes.push(closed[goal]);

    while (!nodes.empty())
    {
        s_node* curr_node = nodes.front();
        nodes.pop();
        if (optimal_traj.find(curr_node->id) == optimal_traj.end())
            optimal_traj[curr_node->id] = curr_node;
        else
            continue;

//        std::cout << curr_node->id << std::endl;


        for (const auto& pair : curr_node->parents) {
            nodes.push(closed[pair.first]);
        }
    }


    if (optimal_traj.size() > closed.size())
    {
        std::cout << "error in astar: no path found "<< start<<","<<goal << std::endl;
        assert(false);
        exit(1);
    }


    std::vector<int> temp_traj;
    std::stack<s_node*> stack_nodes;
    stack_nodes.push(optimal_traj[goal]);
    while (!stack_nodes.empty())
    {
        s_node* curr_node = stack_nodes.top();
        stack_nodes.pop();
        temp_traj.push_back(curr_node->id);
        auto it = curr_node->parents.begin();
        if (it != curr_node->parents.end()) {
            stack_nodes.push(it->second);
        }
    }


    int n_size = temp_traj.size();
    traj.resize(n_size);
    for (int i = 0; i < n_size; i++){
        traj[i] = temp_traj[n_size - 1 - i];
    }

    return *goal_node;
}
}

