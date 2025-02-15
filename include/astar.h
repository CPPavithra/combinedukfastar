#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <unordered_map> 
#include <string>
#include <utility> // For std::pair
#include <functional> // For std::hash
#include <cmath>
#include <deque>
#include "common.h"

using namespace std;

struct Node
{
        float x,y;
        double g_cost,h_cost,f_cost;
        Node *parent;
        //using an initialiser list instead of this keyword
        Node(float x,float y, double g_cost=0,double h_cost=0,Node *parent=NULL): x(x), y(y), g_cost(g_cost), h_cost(h_cost), f_cost(g_cost+h_cost), parent(parent) {}
        bool operator>(const Node& other) const
        {
                return f_cost > other.f_cost;
        }//overloading > to comapre between final cost of the nodes
};

struct comparenode {
    bool operator()(Node*a, Node*b)
    {
            return *a>*b;
    }
};

double heuristic(int x1, int y1, int x2, int y2);

vector<Node>astar(const std::unordered_map<std::pair<int, int>, CellCost, pair_hash>& occupancyGrid, Node start, Node goal);

#endif //ASTAR_H
