#ifndef PATH_HEAPS_H
#define PATH_HEAPS_H
#include "path_common.h"

struct binary_heap
{
    heap_node** Nodes;
    u32 Size;
    u32 Capacity;
};


void SetupBinaryHeap(binary_heap* Heap, u32 InitialSize);
heap_node* AddToHeap(binary_heap* Heap, heap_node* Node);
heap_node* PopFromHeap(binary_heap* Heap);
void UpdateNodeInHeap(binary_heap* Heap, heap_node* Node, f32 Cost);
#endif
