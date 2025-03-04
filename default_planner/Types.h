#pragma once
#include <limits.h>


#include <vector>
#include <iostream>
#include <deque>
#include <regex>
#include <fstream>
#include <cassert>
#include <unordered_set>


#define MAX_TIMESTEP INT_MAX/2

#include "SharedEnv.h"
#include "ActionModel.h"


#include "heap.h"

namespace DefaultPlanner{

	
	enum DONE{
		NOT_DONE,
		DONE
	};

	struct Int4{
		int d[4];
	};

	struct Int2{
		int d[4];
	};

	struct DCR{
		int loc;
		int state;
	};

	typedef std::vector<int> Traj;

	struct PIBT_C{
		int location;
		int heuristic;
		int orientation;  // 0:east, 1:south, 2:west, 3:north
		int tie_breaker;

		//constructor
		PIBT_C(int location, int heuristic, int orientation, int tie_breaker):
			location(location), heuristic(heuristic), orientation(orientation), tie_breaker(tie_breaker) {};
	};

	struct HNode
		{
			int location;
			int g;
            int h;

			unsigned int priority;
			unsigned int get_priority() const { return priority; }
    		void set_priority(unsigned int p) { priority = p; }


			HNode() = default;
			HNode(int location, int g, int h) : location(location), g(g), h(h) {}


		};


//        struct compare_node
//        {
//            bool operator()(const HNode& n1, const HNode& n2) const
//            {
//                return (n1.g + n1.h) < (n2.g + n2.h);
//            }
//        };

    struct compare_node
    {
        bool operator()(const HNode& n1, const HNode& n2) const
        {
            return n1.g < n2.g;
        }
    };


    struct THeuristicTable{

        std::vector<int> traffic_htable;
        pqueue<HNode, compare_node, min_q> traffic_open;
        std::vector<bool> traffic_closed;

        std::unordered_map<int, HNode*> node_list;


    };

    struct HeuristicTable{
		std::vector<int> htable;
		std::deque<HNode> open;


		bool empty(){
			return htable.empty();
		}
	};



	struct d2p{
		int label = 0;
		int id = -1;
		int cost = -1;
		int togo = -1;

		d2p(int label, int id, int cost, int togo):label(label), id(id), cost(cost), togo(togo){};
	};

	struct Dist2Path{
		int label = 0;
		std::vector<d2p> dist2path;
		std::deque<d2p> open;
		
		bool empty(){
			return dist2path.empty();
		}

	};

	typedef std::vector<std::vector<int>> Neighbors;


}







