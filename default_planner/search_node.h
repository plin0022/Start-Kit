#ifndef SEARCH_NODE_H
#define SEARCH_NODE_H

#include <vector>
#include <iostream>
#include <cmath>


namespace DefaultPlanner{
struct s_node
{
    int label = 0;
    int id = -1; //also location, -1 indicated not generated yet.
    float g = 0;
    int h = 0;
    float op_flow = 0;
    float all_vertex_flow = 0;
//    int op_flow = 0;
//    int all_vertex_flow = 0;
    bool closed = false;
    int depth = 0;
    double tie_breaker = 0;
    s_node* parent = nullptr;

    // parents and children
    std::unordered_map<int, s_node*> parents;

    unsigned int priority;

    s_node(int id, int g, int h, float op_flow, int depth) : id(id), g(g), h(h), op_flow(op_flow),depth(depth) {};
//    s_node(int id, int g, int h, int op_flow, int depth) : id(id), g(g), h(h), op_flow(op_flow),depth(depth) {};
    s_node() = default;

    float get_f() const { return g + h; }
    bool is_closed() const { return closed; }
    void close() { closed = true; }


    float get_op_flow() const { return op_flow; }
    float get_all_vertex_flow() const { return all_vertex_flow; }
    void set_all_flow(float op_flow,  float all_vertex_flow){
        this->op_flow = op_flow;
        this->all_vertex_flow = all_vertex_flow;
    };

//    int get_op_flow() const { return op_flow; }
//    int get_all_vertex_flow() const { return all_vertex_flow; }
//    void set_all_flow(int op_flow,  int all_vertex_flow){
//        this->op_flow = op_flow;
//        this->all_vertex_flow = all_vertex_flow;
//    };

    float get_g() const { return g; }
    int get_h() const { return h; }
    unsigned int get_priority() const { return priority; }
    void set_priority(unsigned int p) { priority = p; }

    void reset(){
        label = 0;
        id = -1;
        g = 0;
        h = 0;
        op_flow = 0;
        all_vertex_flow = 0;
        closed = false;
        depth = 0;
        parent = nullptr;
        tie_breaker = 0;

        parents.clear();
    }
    /* data */
};

struct equal_search_node
{
    inline bool operator()(const s_node& lhs, const s_node& rhs) const
    {
        return lhs.id == rhs.id && lhs.get_op_flow() == rhs.get_op_flow() && lhs.get_all_vertex_flow() == rhs.get_all_vertex_flow() && lhs.get_g() == rhs.get_g();
    }
};


struct re_f{
    inline bool operator()(const s_node& lhs, const s_node& rhs) const
    {
        return lhs.get_f() < rhs.get_f();
    }
};

struct re_jam{
    inline bool operator()(const s_node& lhs, const s_node& rhs) const
    {
        if (lhs.tie_breaker  == rhs.tie_breaker)
        {
                return lhs.get_f() < rhs.get_f();
        }
        else
            return lhs.tie_breaker < rhs.tie_breaker;
    }
};

struct cmp_less_f //focal search open
{
    inline bool operator()(const s_node& lhs, const s_node& rhs) const
    {

        if (lhs.get_f() == rhs.get_f()){
                if (lhs.get_g() == rhs.get_g())
                    return rand() % 2;
                else
                    return lhs.get_g() > rhs.get_g();
        }
        else
            return lhs.get_f() < rhs.get_f();

    }
};

struct re_of{ //re-expansion rule astar
    inline bool operator()(const s_node& lhs, const s_node& rhs) const
    {

        return lhs.get_op_flow() + lhs.get_all_vertex_flow() + lhs.get_f() < rhs.get_op_flow()+ rhs.get_all_vertex_flow() + rhs.get_f();

    }
};



struct cmp_less_of //astar open
{
    inline bool operator()(const s_node& lhs, const s_node& rhs) const
    {

            if (lhs.get_op_flow() + lhs.get_all_vertex_flow() + lhs.get_f() == rhs.get_op_flow() + rhs.get_all_vertex_flow() + rhs.get_f())
            {
                if (lhs.get_f()  == rhs.get_f())
                {
                        if (lhs.get_g() == rhs.get_g())
                            if (lhs.tie_breaker  == rhs.tie_breaker)
                                return rand() % 2;
                            else
                                return lhs.tie_breaker < rhs.tie_breaker;
                        else
                            return lhs.get_g() > rhs.get_g();
                }
                else
                    return lhs.get_f() < rhs.get_f();
            }
            else
                return lhs.get_op_flow() + lhs.get_all_vertex_flow() + lhs.get_f() < rhs.get_op_flow()+ rhs.get_all_vertex_flow() + rhs.get_f();

    }
};



// new comparison operator



    struct cmp_less_of_123 //astar open
    {
        inline bool operator()(const s_node& lhs, const s_node& rhs) const
        {
            const float EPSILON = 1e-6;
            float diff = lhs.get_op_flow() + lhs.get_all_vertex_flow() + lhs.get_f() -
                         (rhs.get_op_flow()+ rhs.get_all_vertex_flow() + rhs.get_f());

            return diff < -EPSILON;
            return lhs.get_f() < rhs.get_f();
        }
    };


    struct large_than_fmin {
        inline bool operator()(const s_node& lhs, float fmin) const {
            const float EPSILON = 1e-6;
            float lhs_value = lhs.get_g();
            float diff = lhs_value - fmin;

            return diff > EPSILON;
        }
    };



    struct re_of_float{
        inline bool operator()(const s_node& lhs, const s_node& rhs) const
        {
            const float EPSILON = 1e-6;
            float diff = lhs.get_op_flow() + lhs.get_all_vertex_flow() + lhs.get_f() -
                    (rhs.get_op_flow()+ rhs.get_all_vertex_flow() + rhs.get_f());

            return diff < -EPSILON;

        }
    };

    struct same_of_float{
        inline bool operator()(const s_node& lhs, const s_node& rhs) const
        {
            const float EPSILON = 1e-6;
            float diff = lhs.get_op_flow() + lhs.get_all_vertex_flow() + lhs.get_f() -
                         (rhs.get_op_flow()+ rhs.get_all_vertex_flow() + rhs.get_f());

            return std::fabs(diff) < EPSILON;

        }
    };


    struct small_than_fmin {
        inline bool operator()(const s_node& lhs, float fmin) const {
            const float EPSILON = 1e-6;
            float lhs_value = lhs.get_op_flow() + lhs.get_all_vertex_flow() + lhs.get_f();
            float diff = lhs_value - fmin;

            return diff < -EPSILON;
        }
    };

    struct same_with_fmin {
        inline bool operator()(const s_node& lhs, float fmin) const {
            const float EPSILON = 1e-6;
            float lhs_value = lhs.get_op_flow() + lhs.get_all_vertex_flow() + lhs.get_f();

            return std::fabs(lhs_value - fmin) < EPSILON;
        }
    };


}
#endif