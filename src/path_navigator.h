#ifndef PATH_NAVIGATOR_H
#define PATH_NAVIGATOR_H

#include "path_common.h"
#include "path_heaps.h"
#include <vector>

enum nav_order_method
{
    NavOrder_AStar,
    NavOrder_Dijkstra,
    NavOrder_BreadthFirst,
    NavOrder_Greedy,
};

//TODO(andrew): implement canonical and subgoal
enum nav_expand_method
{
    NavExpand_Immediate,
    NavExpand_JumpPoint,
    NavExpand_Canonical,
    NavExpand_Subgoal
};

struct navigator
{
    heap_node* Nodes;
    binary_heap Heap;
    u32 RunID;
    nav_order_method OrderMethod = NavOrder_AStar;
    nav_expand_method ExpandMethod = NavExpand_Immediate;
    u32 JumpBound = 16;
    u32 StartIndex;
    u32 GoalIndex;

    u32* StartConnected;
    u32 StartCapacity;
    u32 StartSize;

    u32* GoalConnected;
    u32 GoalCapacity;
    u32 GoalSize;

    u32 NumberOfSubgoals;
};

u32 WalkableGridIndex(u32 Index, grid Grid);
void SetupNavigator(navigator* Navigator, grid Grid);
void SetupSubgoalGraph(navigator* Navigator, grid Grid);
u32 Navigate(navigator* Navigator, u32_pair PointA, u32_pair PointB, grid Grid, std::vector<u32>* Path);

#endif
