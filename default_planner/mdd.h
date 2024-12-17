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