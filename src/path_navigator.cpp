#include "path_navigator.h"
#include <math.h>

//TODO(andrew): Fix the broken subgoal logic

#define DIAG_COST 1.41f

#define JPS_NO_JUMP -1

#define CLEAR_N 0
#define CLEAR_S 1
#define CLEAR_W 2 
#define CLEAR_E 3
#define CLEAR_NE 4
#define CLEAR_NW 5 
#define CLEAR_SE 6
#define CLEAR_SW 7 

#define ArrayCount(array) (sizeof(array) / sizeof(array[0]))
static u32* DebugPointerArray[1 << 16];
static u32 DebugArraySize = 0;

static inline u32 WalkableGridIndex(u32 Index, grid Grid)
{
    return Index < Grid.Width * Grid.Height && Grid.Data[Index] == FREE_CELL;
}

static b32 DebugContainsPointer(u32* Pointer)
{
#if PATH_DEBUG
    for(u32 Count=0; Count < DebugArraySize; ++Count)
    {
        if(Pointer == DebugPointerArray[Count])
        {
            return TRUE;
        }
    }

    return FALSE;
#else
    return TRUE;
#endif
}

static u32* NewArray(u32 Size)
{
#if PATH_DEBUG
    Assert(DebugArraySize < ArrayCount(DebugPointerArray));
    u32* Pointer = new u32[Size];
    DebugPointerArray[DebugArraySize++] = Pointer;
    return Pointer;
#else
    return new u32[Size];
#endif
}

static void DeleteArray(u32* Pointer)
{
#if PATH_DEBUG
    for(u32 Count=0; Count < DebugArraySize; ++Count)
    {
        if(Pointer == DebugPointerArray[Count])
        {
            delete[] Pointer;
            DebugPointerArray[Count] = DebugPointerArray[--DebugArraySize];
            return;
        }
    }

    InvalidCodePath;
#else
    delete[] Pointer;
#endif
}

static u32_pair GetDXAndDY(u32 Direction, grid Grid)
{
    u32 DX = 0, DY = 0;
    switch(Direction)
    {
        case CLEAR_N:
            DY = -1;
            break;
        case CLEAR_E:
            DX = 1;
            break;
        case CLEAR_W:
            DX = -1;
            break;
        case CLEAR_S:
            DY = 1;
            break;
        case CLEAR_NE:
            DX = 1;
            DY = 1;
            break;
        case CLEAR_NW:
            DX = -1;
            DY = -1;
            break;
        case CLEAR_SE:
            DX = 1;
            DY = 1;
            break;
        case CLEAR_SW:
            DX = -1;
            DY = 1;
            break;
        default: InvalidCodePath;
    }
    return {DX, DY};
}

static u32 ComputeClearance(navigator* Navigator, u32 XPos, u32 YPos, u32 Direction, u32 Limit, grid Grid)
{
    u32_pair Delta = GetDXAndDY(Direction, Grid);
    u32 Distance = 0;
    if(!Delta.X && !Delta.Y) InvalidCodePath;
    u32 XCurrPos = XPos;
    u32 YCurrPos = YPos;
    for(;Distance < Limit;)
    {
        if(Delta.X && Delta.Y)
        {
            if(!(XCurrPos + Delta.X < Grid.Width && WalkableGridIndex(YCurrPos * Grid.Width + XCurrPos + Delta.X, Grid)) ||
               !(YCurrPos + Delta.Y < Grid.Height && WalkableGridIndex((YCurrPos + Delta.Y) * Grid.Width + XCurrPos, Grid)))
            {
                return Distance;
            }
        }
        u32 NewIndex = (YCurrPos + Delta.Y) * Grid.Width + XCurrPos + Delta.X;
        if(!(XCurrPos + Delta.X < Grid.Width && YCurrPos + Delta.Y < Grid.Height && WalkableGridIndex(NewIndex, Grid)))
        {
            return Distance;
        }

        ++Distance;
        XCurrPos += Delta.X;
        YCurrPos += Delta.Y;

        if(Navigator->Nodes[NewIndex].Connected)
        {
            return Distance;
        }
    }
    return Distance;
}

static u32 GetClearance(navigator* Navigator, heap_node* Node, u32 Direction, u32 Runtime, grid Grid)
{
    u32 TotalClearance = 0;
    if(Runtime)
    {
        u32 Index = Node->GridIndex;
        return ComputeClearance(Navigator, Index % Grid.Width, Index / Grid.Width, Direction, U32_MAX, Grid);
    }
    else
    {
        u32 ClearShortDist = 0;
        u32 CurrentIndex = Node->GridIndex;
        do
        {
            heap_node* CurrentNode = Navigator->Nodes + CurrentIndex;
            u32_pair Delta = GetDXAndDY(Direction, Grid);
            ClearShortDist = Node->Clearances[Direction];
            TotalClearance += ClearShortDist;
            CurrentIndex += (Delta.Y + Delta.X) * ClearShortDist;
        } while(ClearShortDist == 255);
    }
    return TotalClearance;
}

static b32 IsValidSubgoal(u32 Index, grid Grid)
{
    u32 XPos = Index % Grid.Width;
    u32 YPos = Index / Grid.Width;

    u32 NorthFree = YPos - 1 < Grid.Height && WalkableGridIndex(Index-Grid.Width, Grid);
    u32 SouthFree = YPos + 1 < Grid.Height && WalkableGridIndex(Index+Grid.Width, Grid);
    u32 WestFree = XPos - 1 < Grid.Width && WalkableGridIndex(Index-1, Grid);
    u32 EastFree = XPos + 1 < Grid.Width && WalkableGridIndex(Index+1, Grid);

    return (WalkableGridIndex(Index, Grid) && (
      (NorthFree && EastFree && !WalkableGridIndex(Index-Grid.Width+1, Grid)) ||
      (NorthFree && WestFree && !WalkableGridIndex(Index-Grid.Width-1, Grid)) ||
      (SouthFree && EastFree && !WalkableGridIndex(Index+Grid.Width+1, Grid)) || 
      (SouthFree && WestFree && !WalkableGridIndex(Index+Grid.Width-1, Grid)) 
      ));
}

static void SubgoalAddConnectedIndex(navigator* Navigator, heap_node* Node, u32 ConnectedIndex, grid Grid)
{
    Node->Connected[Node->ConnectedSize++] = ConnectedIndex;
    if(Node->ConnectedSize == Node->ConnectedCapacity)
    {
        u32* OldConnected = Node->Connected;
        Node->ConnectedCapacity <<= 1;
        Node->Connected = NewArray(Node->ConnectedCapacity);
#if 0
        for(u32 Count=0; Count < Grid.Width * Grid.Height; ++Count)
        {
            Assert(Node->GridIndex == Count || Node->Connected != Navigator->Nodes[Count].Connected);
        }
		Assert(Node->ConnectedSize < Node->ConnectedCapacity);
#endif
        for(u32 Count=0; Count < Node->ConnectedSize; ++Count)
        {
            Node->Connected[Count] = OldConnected[Count];
        }
        DeleteArray(OldConnected);
    }
}

static void FindSubgoalConnections(navigator* Navigator, heap_node* Node, u32 Direction, b32 Runtime, grid Grid)
{
    u32 ClearDistance = GetClearance(Navigator, Node, Direction, Runtime, Grid);

    if(!ClearDistance)
    {
        return;
    }
    u32_pair Delta = GetDXAndDY(Direction, Grid);
    u32 DYScaled = Delta.Y * Grid.Width;
    u32 PossibleSubgoalIndex = Node->GridIndex + (DYScaled + Delta.X) * ClearDistance;
    if(IsValidSubgoal(PossibleSubgoalIndex, Grid) || PossibleSubgoalIndex == Navigator->StartIndex ||
       PossibleSubgoalIndex == Navigator->GoalIndex)
    {
        SubgoalAddConnectedIndex(Navigator, Node, PossibleSubgoalIndex, Grid);
        ClearDistance = ClearDistance ? ClearDistance - 1 : 0;
    }
    if(Delta.X != 0 && Delta.Y != 0)
    {
        u32 Horiz = ((i32) Delta.X) > 0? CLEAR_E : CLEAR_W;
        u32 Vert = ((i32) Delta.Y) > 0 ? CLEAR_S : CLEAR_N;
        u32 DXMax = GetClearance(Navigator, Node, Horiz, Runtime, Grid);
        u32 DYMax = GetClearance(Navigator, Node, Vert, Runtime, Grid);
        if(Navigator->Nodes[Node->GridIndex + DXMax * Delta.X].Connected)
            --DXMax;
        if(Navigator->Nodes[Node->GridIndex + DYMax * DYScaled].Connected)
            --DYMax;
        for(u32 ClearCount=0; ClearCount <= ClearDistance; ++ClearCount)
        {
            u32 StopIndex = Node->GridIndex + ClearCount * (DYScaled + Delta.X);
            heap_node* StopNode = Navigator->Nodes + StopIndex;
            u32 StopClearX = GetClearance(Navigator, StopNode, Horiz, Runtime, Grid);
            u32 StopClearY = GetClearance(Navigator, StopNode, Vert, Runtime, Grid);
            u32 StopXIndex = StopIndex + StopClearX*Delta.X;
            u32 StopYIndex = StopIndex + StopClearY*DYScaled;

            if(StopClearX <= DXMax && (IsValidSubgoal(StopXIndex, Grid) || 
               Navigator->StartIndex == StopXIndex || Navigator->GoalIndex == StopXIndex))
            {
                SubgoalAddConnectedIndex(Navigator, Node, StopXIndex, Grid);
                --StopClearX;
            }

            DXMax = StopClearX < DXMax ? StopClearX  : DXMax;

            if(StopClearY <= DYMax && (IsValidSubgoal(StopYIndex, Grid) ||
               Navigator->StartIndex == StopYIndex || Navigator->GoalIndex == StopYIndex))
            {
                SubgoalAddConnectedIndex(Navigator, Node, StopYIndex, Grid);
                --StopClearY;
            }

            DYMax = StopClearY < DYMax ? StopClearY : DYMax;
        }
    }
}

static void PrecomputeSubgoalGraph(navigator* Navigator, grid Grid)
{
    std::vector<u32> SubgoalIndices;
    for(u32 Index=0; Index < Grid.Width*Grid.Height; ++Index)
    {
        heap_node* Node = Navigator->Nodes + Index;

        if(IsValidSubgoal(Index, Grid))
        {
            Node->ConnectedCapacity = 4;    
            Node->Connected = NewArray(Node->ConnectedCapacity);
            Node->ConnectedSize = 0;
            SubgoalIndices.push_back(Index);
        }
    }

    for(u32 Index=0; Index < Grid.Width * Grid.Height; ++Index)
    {
        heap_node* Node = Navigator->Nodes + Index;
        u32 XPos = Index % Grid.Width;
        u32 YPos = Index / Grid.Width;
        Node->Clearances[CLEAR_N ] = ComputeClearance(Navigator, XPos, YPos, CLEAR_N,  U8_MAX, Grid);
        Node->Clearances[CLEAR_S ] = ComputeClearance(Navigator, XPos, YPos, CLEAR_S,  U8_MAX, Grid);
        Node->Clearances[CLEAR_W ] = ComputeClearance(Navigator, XPos, YPos, CLEAR_W,  U8_MAX, Grid);
        Node->Clearances[CLEAR_E ] = ComputeClearance(Navigator, XPos, YPos, CLEAR_E,  U8_MAX, Grid);
        Node->Clearances[CLEAR_NE] = ComputeClearance(Navigator, XPos, YPos, CLEAR_NE, U8_MAX, Grid);
        Node->Clearances[CLEAR_NW] = ComputeClearance(Navigator, XPos, YPos, CLEAR_NW, U8_MAX, Grid);
        Node->Clearances[CLEAR_SE] = ComputeClearance(Navigator, XPos, YPos, CLEAR_SE, U8_MAX, Grid);
        Node->Clearances[CLEAR_SW] = ComputeClearance(Navigator, XPos, YPos, CLEAR_SW, U8_MAX, Grid);
    }

    Navigator->NumberOfSubgoals = (u32) SubgoalIndices.size();
    for(u32 SubgoalCount=0; SubgoalCount < Navigator->NumberOfSubgoals; ++SubgoalCount)
    {
        heap_node* Node = Navigator->Nodes + SubgoalIndices[SubgoalCount];
        for(u32 Direction=0; Direction < 8; ++Direction)
        {
            FindSubgoalConnections(Navigator, Node, Direction, FALSE, Grid);
        }
    }
}

void SetupNavigator(navigator* Navigator, grid Grid)
{
    Navigator->Nodes = new heap_node[Grid.Width * Grid.Height];
    SetupBinaryHeap(&Navigator->Heap, 16);
    memset(Navigator->Nodes, 0, sizeof(heap_node) * Grid.Width * Grid.Height);
    for(u32 NodeIndex=0; NodeIndex < Grid.Width * Grid.Height; ++NodeIndex)
    {
        Navigator->Nodes[NodeIndex].GridIndex = NodeIndex;
    }

    Navigator->StartIndex = -1;
    Navigator->StartCapacity = 4;
    Navigator->StartConnected = NewArray(Navigator->StartCapacity);
    Navigator->StartSize = 0;

    Navigator->GoalIndex = -1;
    Navigator->GoalCapacity = 4;
    Navigator->GoalConnected = NewArray(Navigator->GoalCapacity);
    Navigator->GoalSize = 0;

}

void SetupSubgoalGraph(navigator* Navigator, grid Grid)
{
    PrecomputeSubgoalGraph(Navigator, Grid);
}

static f32 OctileCost(u32 IndexA, u32 IndexB, u32 Width)
{
    u32 AX = IndexA % Width;
    u32 AY = IndexA / Width;
    u32 BX = IndexB % Width;
    u32 BY = IndexB / Width;

    u32 DX = AX > BX ? AX - BX : BX - AX;
    u32 DY = AY > BY ? AY - BY : BY - AY;
    u32 Min = DX > DY ? DY : DX;
    return (DX + DY) + (DIAG_COST - 2) * Min;
}

static void BacktrackPath(navigator* Navigator, u32 IndexA, u32 IndexB, std::vector<u32>* Path)
{
    u32 Index = IndexB;
    do
    {
        Path->push_back(Index);
        Index = Navigator->Nodes[Index].ParentIndex;
    } while(Index != IndexA);
}

static void GetImmediateNeighbors(u32 GridIndex, grid Grid, std::vector<u32>* Neighbors)
{
    u32 XPos = GridIndex % Grid.Width;
    u32 YPos = GridIndex / Grid.Width;
    u32 NorthFree = YPos - 1 < Grid.Height && WalkableGridIndex(GridIndex-Grid.Width, Grid);
    u32 SouthFree = YPos + 1 < Grid.Height && WalkableGridIndex(GridIndex+Grid.Width, Grid);
    u32 WestFree = XPos - 1 < Grid.Width && WalkableGridIndex(GridIndex-1, Grid);
    u32 EastFree = XPos + 1 < Grid.Width && WalkableGridIndex(GridIndex+1, Grid);
    
    if(NorthFree)
        Neighbors->push_back(GridIndex - Grid.Width);
    if(SouthFree)
        Neighbors->push_back(GridIndex + Grid.Width);
    if(WestFree)
        Neighbors->push_back(GridIndex - 1);
    if(EastFree)
        Neighbors->push_back(GridIndex + 1);

    if(NorthFree && WestFree && WalkableGridIndex(GridIndex - Grid.Width - 1, Grid))
        Neighbors->push_back(GridIndex - Grid.Width - 1);

    if(NorthFree && EastFree && WalkableGridIndex(GridIndex - Grid.Width + 1, Grid))
        Neighbors->push_back(GridIndex - Grid.Width + 1);

    if(SouthFree && WestFree && WalkableGridIndex(GridIndex + Grid.Width - 1, Grid))
        Neighbors->push_back(GridIndex + Grid.Width - 1);

    if(SouthFree && EastFree && WalkableGridIndex(GridIndex + Grid.Width + 1, Grid))
        Neighbors->push_back(GridIndex + Grid.Width + 1);
}

static u32 JumpStraight(u32 GridIndex, u32 GoalIndex, u32 Dir, u32 JumpBound, grid Grid)
{
    u32 IsHorizJump = Dir == 1 || Dir == -1;
    u32 JumpPoint = GridIndex;
    u32 DYNorm = (i32) Dir > 0 ? 1 : -1;
    for(u32 JumpIterate = 0; JumpPoint != GoalIndex && JumpIterate < JumpBound; ++JumpIterate)
    {
        u32 XPos = JumpPoint % Grid.Width;
        u32 YPos = JumpPoint / Grid.Width;
        u32 DXWalk = (XPos + Dir) < Grid.Width && WalkableGridIndex(JumpPoint+Dir, Grid);
        u32 DYWalk = (YPos + DYNorm) < Grid.Height && WalkableGridIndex(JumpPoint+Dir, Grid);

        u32 NorthFree = YPos - 1 < Grid.Height && WalkableGridIndex(JumpPoint-Grid.Width, Grid);
        u32 SouthFree = YPos + 1 < Grid.Height && WalkableGridIndex(JumpPoint+Grid.Width, Grid);
        u32 WestFree = XPos - 1 < Grid.Width && WalkableGridIndex(JumpPoint-1, Grid);
        u32 EastFree = XPos + 1 < Grid.Width && WalkableGridIndex(JumpPoint+1, Grid);

        u32 SouthBlocked = NorthFree && !WalkableGridIndex(JumpPoint-Grid.Width-Dir, Grid);
        u32 NorthBlocked = SouthFree && !WalkableGridIndex(JumpPoint+Grid.Width-Dir, Grid);
        u32 WestBlocked = WestFree && !WalkableGridIndex(JumpPoint-1-Dir, Grid);
        u32 EastBlocked = EastFree && !WalkableGridIndex(JumpPoint+1-Dir, Grid);
        if((IsHorizJump && (SouthBlocked || NorthBlocked)) ||
           (!IsHorizJump && (WestBlocked || EastBlocked)))
        {
            break;
        }

        u32 DeeperJump = (IsHorizJump && DXWalk) || (!IsHorizJump && DYWalk);
        if(!DeeperJump)
            return JPS_NO_JUMP;

        JumpPoint += Dir;
    }
    return JumpPoint;
}

static u32 JumpDiagonal(u32 GridIndex, u32 GoalIndex, u32 DX, u32 DY, u32 JumpBound, grid Grid)
{
    u32 JumpPoint = GridIndex;
    u32 DYNorm = (i32) DY > 0 ? 1 : -1;
    for(u32 JumpIterate = 0; JumpPoint != GoalIndex && JumpIterate < JumpBound; ++JumpIterate)
    {
        u32 XPos = JumpPoint % Grid.Width;
        u32 YPos = JumpPoint / Grid.Width;
        u32 DXWalk = (XPos + DX) < Grid.Width && WalkableGridIndex(JumpPoint+DX, Grid);
        u32 DYWalk = (YPos + DYNorm) < Grid.Height && WalkableGridIndex(JumpPoint+DY, Grid);

        if((DXWalk && JumpStraight(JumpPoint+DX, GoalIndex, DX, JumpBound, Grid) != JPS_NO_JUMP) ||
           (DYWalk && JumpStraight(JumpPoint+DY, GoalIndex, DY, JumpBound, Grid) != JPS_NO_JUMP))
        {
            break;
        }

        if(!(DXWalk && DYWalk && WalkableGridIndex(JumpPoint+DX+DY, Grid)))
            return JPS_NO_JUMP;

        JumpPoint = JumpPoint+ DX + DY;
    }
    return JumpPoint;
}

static void TryAddingJump(u32 JumpPoint, std::vector<u32>* Neighbors)
{
    if(JumpPoint != JPS_NO_JUMP)
        Neighbors->push_back(JumpPoint);
}


static void  GetJumpPoints(
    u32 GridIndex, 
    u32 ParentIndex,
    u32 GoalIndex, 
    u32 JumpBound,
    grid Grid, 
    std::vector<u32>* Neighbors)
{
    u32 XPos = GridIndex % Grid.Width;
    u32 YPos = GridIndex / Grid.Width;

    u32 DX = (XPos - ParentIndex % Grid.Width);
    u32 DY = (YPos - ParentIndex / Grid.Width);
    if(DX)
        DX = (i32) DX > 0 ? 1 : -1;
    if(DY)
        DY = (i32) DY > 0 ? 1 : -1;

    u32 DXWalk = (XPos + DX) < Grid.Width && WalkableGridIndex(GridIndex+DX, Grid);
    u32 DYWalk = (YPos + DY) < Grid.Height && WalkableGridIndex(GridIndex+DY*Grid.Width, Grid);
    u32 DYUnscaled = DY;
    DY *= Grid.Width;
    if(DX && DY)
    {
        if(DXWalk && DYWalk && WalkableGridIndex(GridIndex+DX+DY, Grid))
        {
            TryAddingJump(JumpDiagonal(GridIndex+DX+DY, GoalIndex, DX, DY, JumpBound, Grid), Neighbors);
        }

        if(DYWalk && WalkableGridIndex(GridIndex+DY, Grid))
        {
            TryAddingJump(JumpStraight(GridIndex+DY, GoalIndex, DY, JumpBound, Grid), Neighbors);
        }

        if(DXWalk && WalkableGridIndex(GridIndex+DX, Grid))
        {
            TryAddingJump(JumpStraight(GridIndex+DX, GoalIndex, DX, JumpBound, Grid), Neighbors);
        }
    }
    else if(DX)
    {
        u32 NorthFree = YPos - 1 < Grid.Height && WalkableGridIndex(GridIndex - Grid.Width, Grid);
        u32 SouthFree = YPos + 1 < Grid.Height && WalkableGridIndex(GridIndex + Grid.Width, Grid);
        if(DXWalk)
        {
            TryAddingJump(JumpStraight(GridIndex+DX, GoalIndex, DX, JumpBound, Grid), Neighbors);
        }

        if(XPos - DX < Grid.Width && NorthFree && !WalkableGridIndex(GridIndex-Grid.Width-DX, Grid))
        {
            TryAddingJump(JumpStraight(GridIndex-Grid.Width, GoalIndex, 0-Grid.Width, JumpBound, Grid), Neighbors);
            if(DXWalk && WalkableGridIndex(GridIndex+DX-Grid.Width, Grid))
                TryAddingJump(JumpDiagonal(GridIndex+DX-Grid.Width, GoalIndex, DX, 0-Grid.Width, JumpBound, Grid), Neighbors);
        }

        if(XPos - DX < Grid.Width && SouthFree && !WalkableGridIndex(GridIndex+Grid.Width-DX, Grid))
        {
            TryAddingJump(JumpStraight(GridIndex+Grid.Width, GoalIndex, Grid.Width, JumpBound, Grid), Neighbors);
            if(DXWalk && WalkableGridIndex(GridIndex+DX+Grid.Width, Grid))
                TryAddingJump(JumpDiagonal(GridIndex+DX+Grid.Width, GoalIndex, DX, Grid.Width, JumpBound, Grid), Neighbors);
        }
    }
    else
    {
        u32 WestFree = XPos - 1 < Grid.Width && WalkableGridIndex(GridIndex - 1, Grid);
        u32 EastFree = XPos + 1 < Grid.Width && WalkableGridIndex(GridIndex + 1, Grid);
        if(DYWalk)
        {
            TryAddingJump(JumpStraight(GridIndex+DY, GoalIndex,  DY, JumpBound, Grid), Neighbors);
        }

        if(YPos - DYUnscaled < Grid.Height && WestFree && !WalkableGridIndex(GridIndex-DY-1, Grid))
        {
            TryAddingJump(JumpStraight(GridIndex-1, GoalIndex, -1, JumpBound, Grid), Neighbors);
            if(DYWalk && WalkableGridIndex(GridIndex+DY-1, Grid))
                TryAddingJump(JumpDiagonal(GridIndex+DY-1, GoalIndex, -1, DY, JumpBound, Grid), Neighbors);
        }

        if(YPos - DYUnscaled < Grid.Height && EastFree && !WalkableGridIndex(GridIndex-DY+1, Grid))
        {
            TryAddingJump(JumpStraight(GridIndex+1, GoalIndex, 1, JumpBound, Grid), Neighbors);
            if(WalkableGridIndex(GridIndex+DY+1, Grid))
                TryAddingJump(JumpDiagonal(GridIndex+DY+1, GoalIndex, 1, DY, JumpBound, Grid), Neighbors);
        }
    }
}


static inline void GetConnected(navigator* Navigator, u32 GridIndex, grid Grid, std::vector<u32>*Neighbors)
{
    heap_node* Node = Navigator->Nodes + GridIndex;
    Assert(DebugContainsPointer(Node->Connected));
    for(u32 ConnectedCount=0; ConnectedCount < Node->ConnectedSize; ++ConnectedCount)
    {
        Neighbors->push_back(Node->Connected[ConnectedCount]);
    }
}

static void DoNodeExpansion
    (navigator* Navigator, 
     u32 GridIndex, 
     u32 StartIndex,
     u32 GoalIndex, 
     grid Grid, 
     std::vector<u32>* Neighbors)
{

    switch(Navigator->ExpandMethod)
    {
        case NavExpand_Immediate:
            GetImmediateNeighbors(GridIndex, Grid, Neighbors);
            break;
        case NavExpand_JumpPoint:
            if(Navigator->Nodes[GridIndex].ParentIndex == GridIndex)
            {
                std::vector<u32> NeighborParents;
                GetImmediateNeighbors(GridIndex, Grid, &NeighborParents);
                for(u32 NeighborIndex : NeighborParents)
                    GetJumpPoints(NeighborIndex, GridIndex, GoalIndex, Navigator->JumpBound, Grid, Neighbors);
            }
            else
            {
                GetJumpPoints(GridIndex, Navigator->Nodes[GridIndex].ParentIndex, 
                              GoalIndex, Navigator->JumpBound, Grid, Neighbors);
            }
            break;
        case NavExpand_Canonical:
            if(Navigator->Nodes[GridIndex].ParentIndex == GridIndex)
            {
                GetImmediateNeighbors(GridIndex, Grid, Neighbors);
            }
            else
            {
                GetJumpPoints(GridIndex, Navigator->Nodes[GridIndex].ParentIndex, GoalIndex, 0, Grid, Neighbors);
            }
            break;
        case NavExpand_Subgoal:
            GetConnected(Navigator, GridIndex, Grid, Neighbors);
            break;
        default:
            GetImmediateNeighbors(GridIndex, Grid, Neighbors);
            break;
    }
}

void UseDijkstraOrder(navigator* Navigator, heap_node* Neighbor, heap_node* Node, u32 GoalIndex, grid Grid)
{
    f32 NewGCost = OctileCost(Node->GridIndex, Neighbor->GridIndex, Grid.Width) + Node->GCost;
    if(Neighbor->OpenID != Navigator->RunID)
    {
        Neighbor->OpenID = Navigator->RunID;
        Neighbor->GCost = NewGCost;
        Neighbor->Cost = NewGCost;
        Neighbor->ParentIndex = Node->GridIndex;
        AddToHeap(&Navigator->Heap, Navigator->Nodes + Neighbor->GridIndex);
    }
    else if (Neighbor->GCost > NewGCost)
    {
        Neighbor->ParentIndex = Node->GridIndex;
        Neighbor->GCost = NewGCost;
        UpdateNodeInHeap(&Navigator->Heap, Neighbor, Neighbor->GCost);
    }
}


void UseGreedyOrder(navigator* Navigator, heap_node* Neighbor, heap_node* Node, u32 GoalIndex, grid Grid)
{
    if(Neighbor->OpenID != Navigator->RunID)
    {
        Neighbor->OpenID = Navigator->RunID;
        Neighbor->Cost = OctileCost(Neighbor->GridIndex, GoalIndex, Grid.Width);
        Neighbor->ParentIndex = Node->GridIndex;
        AddToHeap(&Navigator->Heap, Navigator->Nodes + Neighbor->GridIndex);
    }
}

void UseAStarOrder(navigator* Navigator, heap_node* Neighbor, heap_node* Node, u32 GoalIndex, grid Grid)
{
    f32 NewGCost = OctileCost(Node->GridIndex, Neighbor->GridIndex, Grid.Width) + Node->GCost;
    if(Neighbor->OpenID != Navigator->RunID)
    {
        Neighbor->OpenID = Navigator->RunID;
        Neighbor->GCost = NewGCost;
        Neighbor->Cost = NewGCost + OctileCost(Neighbor->GridIndex, GoalIndex, Grid.Width);
        Neighbor->ParentIndex = Node->GridIndex;
        AddToHeap(&Navigator->Heap, Navigator->Nodes + Neighbor->GridIndex);
    }
    else if (Neighbor->GCost > NewGCost)
    {
        Neighbor->ParentIndex = Node->GridIndex;
        f32 NewCost = Neighbor->Cost + NewGCost - Neighbor->GCost;
        Neighbor->GCost = NewGCost;
        UpdateNodeInHeap(&Navigator->Heap, Neighbor, NewCost);
    }
}

void UseBreadthFirstOrder(navigator* Navigator, heap_node* Neighbor, heap_node* Node, u32 GoalIndex, grid Grid)
{
    if(Neighbor->OpenID != Navigator->RunID)
    {
        Neighbor->OpenID = Navigator->RunID;
        Neighbor->Cost = 1 + Node->Cost;
        Neighbor->ParentIndex = Node->GridIndex;
        AddToHeap(&Navigator->Heap, Navigator->Nodes + Neighbor->GridIndex);
    }
}

void DoNodeOrdering(navigator* Navigator, heap_node* Neighbor, heap_node* Node, u32 GoalIndex, grid Grid)
{
    if(Neighbor->CloseID == Navigator->RunID)
        return;
    switch(Navigator->OrderMethod)
    {
        case NavOrder_Dijkstra:
            UseDijkstraOrder(Navigator, Neighbor, Node, GoalIndex, Grid);
            break;
        case NavOrder_Greedy:
            UseGreedyOrder(Navigator, Neighbor, Node, GoalIndex, Grid);
            break;
        case NavOrder_BreadthFirst:
            UseBreadthFirstOrder(Navigator, Neighbor, Node, GoalIndex, Grid);
        case NavOrder_AStar:
            UseAStarOrder(Navigator, Neighbor, Node, GoalIndex, Grid);
            break;
        default:
            UseAStarOrder(Navigator, Neighbor, Node, GoalIndex, Grid);
            break;
    }
}

static void ConnectStartAndGoalNode(navigator* Navigator, u32 StartIndex, u32 GoalIndex, grid Grid)
{
    if(Navigator->ExpandMethod != NavExpand_Subgoal)
        return;

    Navigator->StartIndex = StartIndex;
    Navigator->GoalIndex = GoalIndex;
    heap_node* Start = Navigator->Nodes + StartIndex;
    heap_node* Goal = Navigator->Nodes + GoalIndex;
    if(!IsValidSubgoal(StartIndex, Grid))
    {
        Assert(DebugContainsPointer(Navigator->StartConnected));
        Start->Connected = Navigator->StartConnected;
        Start->ConnectedCapacity= Navigator->StartCapacity;
        Start->ConnectedSize= 0;
        for(u32 Direction=0; Direction < 8; ++Direction)
        {
            FindSubgoalConnections(Navigator, Start, Direction, TRUE, Grid);
        }
    }

    if(!IsValidSubgoal(GoalIndex, Grid))
    {
        Assert(DebugContainsPointer(Navigator->GoalConnected));
        Goal->Connected = Navigator->GoalConnected;
        Goal->ConnectedCapacity = Navigator->GoalCapacity;
        Goal->ConnectedSize = 0;
        for(u32 Direction=0; Direction < 8; ++Direction)
        {
            FindSubgoalConnections(Navigator, Goal, Direction, TRUE, Grid);
        }
        for(u32 Count = 0; Count < Goal->ConnectedSize; ++Count)
        {
            heap_node* Former = Navigator->Nodes + Goal->Connected[Count];
            SubgoalAddConnectedIndex(Navigator, Former, GoalIndex, Grid);
        }
    }
}

static void RemoveStartAndGoalNode(navigator* Navigator, u32 StartIndex, u32 GoalIndex, grid Grid)
{
    if(Navigator->ExpandMethod != NavExpand_Subgoal)
        return;

    heap_node* Start = Navigator->Nodes + StartIndex;
    heap_node* Goal = Navigator->Nodes + GoalIndex;
    if(!IsValidSubgoal(StartIndex, Grid))
    {
        Assert(DebugContainsPointer(Start->Connected));
        Navigator->StartConnected = Start->Connected;
        Navigator->StartCapacity= Start->ConnectedCapacity;
        Navigator->StartSize = 0;
        Start->Connected = 0;
        Start->ConnectedSize = 0;
        Start->ConnectedCapacity = 0;
    }

    if(!IsValidSubgoal(GoalIndex, Grid))
    {
        Assert(DebugContainsPointer(Goal->Connected));
        Navigator->GoalConnected = Goal->Connected;
        Navigator->GoalCapacity= Goal->ConnectedCapacity;
        Navigator->GoalSize = 0;
        for(u32 Count = 0; Count < Goal->ConnectedSize; ++Count)
        {
            heap_node* Former = Navigator->Nodes + Goal->Connected[Count];
            Former->ConnectedSize--;
        }
        Goal->Connected = 0;
        Goal->ConnectedSize = 0;
        Goal->ConnectedCapacity = 0;
    }
}

u32 Navigate(navigator* Navigator, u32_pair PointA, u32_pair PointB, grid Grid, std::vector<u32>* Path)
{ 
    b32 Result = FALSE;
    std::vector<u32> Neighbors;
    u32 IndexA = PointA.X + PointA.Y * Grid.Width;
    u32 IndexB = PointB.X + PointB.Y * Grid.Width;
    Assert(IndexA < Grid.Width * Grid.Height);
    Assert(IndexB < Grid.Width * Grid.Height);
    ConnectStartAndGoalNode(Navigator, IndexA, IndexB, Grid);
    ++Navigator->RunID;
    Navigator->Nodes[IndexA].Cost = 0;
    Navigator->Nodes[IndexA].GCost = 0;
    Navigator->Nodes[IndexA].OpenID = Navigator->RunID;
    Navigator->Nodes[IndexA].ParentIndex = IndexA;
    Navigator->Heap.Size = 0;
    AddToHeap(&Navigator->Heap, Navigator->Nodes + IndexA);
    while(Navigator->Heap.Size)
    {
        heap_node* Node = PopFromHeap(&Navigator->Heap);
        if(Node->GridIndex == IndexB)
        {
            BacktrackPath(Navigator, IndexA, IndexB, Path);
            Result = TRUE;
        }

        Node->CloseID = Navigator->RunID;
        DoNodeExpansion(Navigator, Node->GridIndex, IndexA, IndexB, Grid, &Neighbors);
        for(u32 NeighborIndex : Neighbors)
        {
            DoNodeOrdering(Navigator, Navigator->Nodes + NeighborIndex, Node, IndexB, Grid);
        }
        Neighbors.clear();
    }
    RemoveStartAndGoalNode(Navigator, IndexA, IndexB, Grid);
    return Result;
}
