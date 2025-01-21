


#include "search.h"


namespace DefaultPlanner{
std::chrono::nanoseconds t;
//a astar minimized the opposide traffic flow with existing traffic flow

    s_node astar(SharedEnvironment* env, std::vector<std::vector<std::pair<bool, std::array<bool, 4>>>>& constraint_flow,
                 std::vector<Int4>& flow,
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

                // find out which edge and reverse edge
                diff = next - curr->id;
                d = get_d(diff,env);
                rev_d = (d < 2) ? (d + 2) : (d - 2);


                // dynamic obstacle
                if (constraint_flow[curr->curr_timestep + curr->get_g() + 1][next].first ||  // vertex conflict
                    constraint_flow[curr->curr_timestep + curr->get_g()][next].second[rev_d])  // edge conflict
                {
                    op_flow = INT_MAX / 10;
                }
                else
                {
                    op_flow = 0;
                }



                cost = curr->g+1;

                assert(next >= 0 && next < env->map.size());
                depth = curr->depth + 1;



                if(ht.empty())
                    h = manhattanDistance(next,goal,env);
                else
                    h = get_heuristic(ht,env, next, ns);


                diff = next - curr->id;
                d = get_d(diff,env);
                if (curr->parent != nullptr){
                    p_diff = curr->id - curr->parent->id;
                    p_d = get_d(p_diff,env);
                    if (p_d!=d)
                        tie_breaker = 0.1;
                    else
                        tie_breaker = 0;
                    //tie breaking on prefering moving forward
                }





//                temp_op = ( (flow[curr->id].d[d]+1) * flow[next].d[(d+2)%4]);///( ( (flow[curr->id].d[d]+1) + flow[next].d[(d+2)%4]));
//
//                //all vertex flow
//                //the sum of all out going edge flow is the same as the total number of vertex visiting.
//                temp_vertex = 1;
//                for (int j=0; j<4; j++){
//                    temp_vertex += flow[next].d[j];
//                }
//
//                op_flow += temp_op;
//
//                all_vertex_flow+= (temp_vertex-1) /2;




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


        if (goal_node == nullptr){
            s_node* false_node = new s_node(-1, env->curr_timestep, 0, 0, 0, 0);
            return *false_node;
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
        curr->close();

        if (curr->id == goal){
            goal_node = curr;
            break;
        }
        expanded++;
        getNeighborLocs(ns,neighbors,curr->id);

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
            if (curr->parent != nullptr){
                p_diff = curr->id - curr->parent->id;
                p_d = get_d(p_diff,env);
                if (p_d!=d)
                    tie_breaker = 0.1;
                else
                    tie_breaker = 0;
                //tie breaking on prefering moving forward
            }


            temp_op = ( (flow[curr->id].d[d]+1) * flow[next].d[(d+2)%4]);///( ( (flow[curr->id].d[d]+1) + flow[next].d[(d+2)%4]));

            //all vertex flow
            //the sum of all out going edge flow is the same as the total number of vertex visiting.
            temp_vertex = 1;
            for (int j=0; j<4; j++){
                temp_vertex += flow[next].d[j];
            }

            op_flow += temp_op;

            all_vertex_flow+= (temp_vertex-1) /2;

            p_diff = 0;
            if (curr->parent != nullptr){
                p_diff = curr->id - curr->parent->id;
            }

            op_flow += curr->op_flow; //op_flow is contra flow
            all_vertex_flow += curr->all_vertex_flow;

            s_node temp_node(next,cost,h,op_flow, depth);
            temp_node.tie_breaker = tie_breaker;
            temp_node.set_all_flow(op_flow,  all_vertex_flow);

            if (!mem.has_node(next)){
                s_node* next_node = mem.generate_node(next,cost,h,op_flow, depth,all_vertex_flow);
                next_node->parent = curr;
                next_node->tie_breaker = tie_breaker;
                open.push(next_node);
                generated++;
            }
            else{
                s_node* existing = mem.get_node(next);

                if (!existing->is_closed()){
                    if (re(temp_node,*existing)){
                        existing->g = cost;
                        existing->parent = curr;
                        existing->depth = depth;
                        existing->tie_breaker = tie_breaker;
                        existing->set_all_flow(op_flow,  all_vertex_flow);
                        open.decrease_key(existing);
                    }
                }
                else{

                    if (re(temp_node,*existing)){
                        std::cout << "error in astar: re-expansion" << std::endl;
                        assert(false);
                        exit(1);
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

    traj.resize(goal_node->depth+1);
    s_node* curr = goal_node;
    for (int i=goal_node->depth; i>=0; i--){
        traj[i] = curr->id;
        curr = curr->parent;
    }

    return *goal_node;
}


}

