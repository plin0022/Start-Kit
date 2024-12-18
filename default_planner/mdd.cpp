//
// Created by lpq66 on 18/12/2024.
// Reference https://github.com/Jiaoyang-Li/CBSH2-RTC
#include "MDD.h"


bool MDD::buildMDD(const ConstraintTable& ct, int num_of_levels, const SingleAgentSolver* _solver, int start, int goal)
{
    auto root = new MDDNode(start, nullptr); // Root
    std::queue<MDDNode*> open;
    list<MDDNode*> closed;
    std::unordered_set<int> visited;
    open.push(root);
    visited.insert(root->location);


    while (!open.empty())
    {
        auto curr = open.front();
        open.pop();
        closed.push_back(curr);


        list<int> next_locations = solver->getNextLocations(curr->location);  // only need the nodes on traffic costs optimal paths

        for (int next_location : next_locations)
        {
            // the node has been added into open before and has different parent
            if ((visited.find(next_location) != visited.end()) && (different parents))
            {
                add parent into parents;
            }
            else
            {
                auto childNode = new MDDNode(next_location, curr);
                open.push(childNode);
            }
        }
    }

    assert(levels.back().size() == 1);

    // Backward
    auto goal_node = levels.back().back();
    MDDNode* del = nullptr;
    for (auto parent : goal_node->parents)
    {
        if (parent->location == goal_node->location) // the parent of the goal node should not be at the goal location
        {
            del = parent;
            continue;
        }
        levels[num_of_levels - 2].push_back(parent);
        parent->children.push_back(goal_node); // add forward edge
    }
    if (del != nullptr)
        goal_node->parents.remove(del);
    for (int t = num_of_levels - 2; t > 0; t--)
    {
        for (auto node : levels[t])
        {
            for (auto parent : node->parents)
            {
                if (parent->children.empty()) // a new node
                {
                    levels[t - 1].push_back(parent);
                }
                parent->children.push_back(node); // add forward edge
            }
        }
    }

    // Delete useless nodes (nodes who don't have any children)
    for (auto it : closed)
        if (it->children.empty() && it->level < num_of_levels - 1)
            delete it;
    closed.clear();
    return true;
}