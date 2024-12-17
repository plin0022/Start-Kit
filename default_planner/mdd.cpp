//
// Created by lpq66 on 18/12/2024.
// Reference https://github.com/Jiaoyang-Li/CBSH2-RTC
bool MDD::buildMDD(const ConstraintTable& ct, int num_of_levels, const SingleAgentSolver* _solver)
{
    this->solver = _solver;
    auto root = new MDDNode(solver->start_location, nullptr); // Root
    root->cost = num_of_levels - 1;
    std::queue<MDDNode*> open;
    list<MDDNode*> closed;
    open.push(root);
    closed.push_back(root);
    levels.resize(num_of_levels);
    while (!open.empty())
    {
        auto curr = open.front();
        open.pop();
        // Here we suppose all edge cost equals 1
        if (curr->level == num_of_levels - 1)
        {
            levels.back().push_back(curr);
            assert(open.empty());
            break;
        }
        // We want (g + 1)+h <= f = numOfLevels - 1, so h <= numOfLevels - g - 2. -1 because it's the bound of the children.
        int heuristicBound = num_of_levels - curr->level - 2;
        list<int> next_locations = solver->getNextLocations(curr->location);
        for (int next_location : next_locations) // Try every possible move. We only add backward edges in this step.
        {
            if (solver->my_heuristic[next_location] <= heuristicBound &&
                !ct.constrained(next_location, curr->level + 1) &&
                !ct.constrained(curr->location, next_location, curr->level + 1)) // valid move
            {
                auto child = closed.rbegin();
                bool find = false;
                for (; child != closed.rend() && ((*child)->level == curr->level + 1); ++child)
                {
                    if ((*child)->location == next_location) // If the child node exists
                    {
                        (*child)->parents.push_back(curr); // then add corresponding parent link and child link
                        find = true;
                        break;
                    }
                }
                if (!find) // Else generate a new mdd node
                {
                    auto childNode = new MDDNode(next_location, curr);
                    childNode->cost = num_of_levels - 1;
                    open.push(childNode);
                    closed.push_back(childNode);
                }
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