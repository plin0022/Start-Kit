//
// Created by lpq66 on 18/12/2024.
//

#ifndef LIFELONG_MDD_H
#define LIFELONG_MDD_H

#endif //LIFELONG_MDD_H



class MDDNode
{
public:
    MDDNode(int currloc, MDDNode* parent)
    {
        location = currloc;
        if (parent != nullptr)
            parents.push_back(parent);
    }
    int location;
    int cost=0; // minimum cost of path traversing this MDD node

    bool operator==(const MDDNode& node) const
    {
        return (this->location == node.location);
    }


    list<MDDNode*> children;
    list<MDDNode*> parents;
};


class MDD
{
private:
    const SingleAgentSolver* solver;

public:
    vector<MDDNode*> nodes;  // store all the nodes on MDD and connections are built

    bool buildMDD(const ConstraintTable& ct,
                  int num_of_levels, const SingleAgentSolver* solver);
    // bool buildMDD(const std::vector <std::list< std::pair<int, int> > >& constraints, int numOfLevels,
    // 	int start_location, const int* moves_offset, const std::vector<int>& my_heuristic, int map_size, int num_col);
    void printNodes() const;
    MDDNode* find(int location, int level) const;
    void deleteNode(MDDNode* node);
    void clear();
    // bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >& cons) const;

    void increaseBy(const ConstraintTable& ct, int dLevel, SingleAgentSolver* solver);
    MDDNode* goalAt(int level);

    MDD() = default;
    MDD(const MDD& cpy);
    ~MDD();
};