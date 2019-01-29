#include "path_heaps.h"

void SetupBinaryHeap(binary_heap* Heap, u32 InitialSize)
{
    Heap->Capacity = InitialSize;
    Heap->Size = 0;
    Heap->Nodes = new heap_node*[Heap->Capacity];
    memset(Heap->Nodes, 0, sizeof(heap_node*) * Heap->Capacity);
}

void DoubleHeap(binary_heap* Heap)
{
    heap_node** OldNodes = Heap->Nodes;
    Heap->Capacity <<= 1;
    Heap->Nodes = new heap_node*[Heap->Capacity];
    for(u32 HeapIndex = 0; HeapIndex < Heap->Size; ++HeapIndex)
    {
        Heap->Nodes[HeapIndex] = OldNodes[HeapIndex];
    }
    delete[] OldNodes;
}

u32 BubbleUp(binary_heap* Heap, u32 HeapIndex)
{
    while(HeapIndex)
    {
        if(Heap->Nodes[HeapIndex/2]->Cost > Heap->Nodes[HeapIndex]->Cost)
        {
            heap_node* Temp = Heap->Nodes[HeapIndex/2];
            Heap->Nodes[HeapIndex/2] = Heap->Nodes[HeapIndex];
            Heap->Nodes[HeapIndex/2]->HeapIndex = HeapIndex/2;
            Heap->Nodes[HeapIndex] = Temp;
            Heap->Nodes[HeapIndex]->HeapIndex = HeapIndex;
            HeapIndex >>= 1;
        } else break;
    }
    return HeapIndex;
}

heap_node* AddToHeap(binary_heap* Heap, heap_node* Node) 
{
    Node->HeapIndex = Heap->Size;
    Heap->Nodes[Heap->Size++] = Node;
    if(Heap->Capacity <= Heap->Size)
    {
        DoubleHeap(Heap);
    }
    BubbleUp(Heap, Node->HeapIndex);
    return Node;
}

heap_node* PopFromHeap(binary_heap* Heap)
{
    heap_node* Node = Heap->Nodes[0];
    Heap->Nodes[0] = Heap->Nodes[--Heap->Size];
    Heap->Nodes[0]->HeapIndex = 0;
    u32 HeapIndex = 0;
    while(Heap->Size > HeapIndex*2+1)
    {
        u32 ChildHeapIndex = Heap->Size > HeapIndex*2+2 && Heap->Nodes[HeapIndex*2+1]->Cost > Heap->Nodes[HeapIndex*2+2]->Cost ? HeapIndex*2+2 : HeapIndex*2+1;
        if(Heap->Nodes[HeapIndex]->Cost > Heap->Nodes[ChildHeapIndex]->Cost)
        {
            heap_node* Temp = Heap->Nodes[HeapIndex];
            Heap->Nodes[HeapIndex] = Heap->Nodes[ChildHeapIndex];
            Heap->Nodes[ChildHeapIndex] = Temp;
            Heap->Nodes[HeapIndex]->HeapIndex = HeapIndex;
            Heap->Nodes[ChildHeapIndex]->HeapIndex = ChildHeapIndex;
            HeapIndex = ChildHeapIndex;
        } else break;
    }
    return Node;
}

void UpdateNodeInHeap(binary_heap* Heap, heap_node* Node, f32 Cost)
{
    if(Node->Cost > Cost)
        Node->Cost = Cost;
    BubbleUp(Heap, Node->HeapIndex);
}
