#ifndef PATH_COMMON_H
#define PATH_COMMON_H

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <vector>

#define PATH_DEBUG 1

#define BLOCKED_CELL 100
#define FREE_CELL 0
#define InvalidCodePath (* (int*) 0 = 0)

#if PATH_DEBUG
#define Assert(expression) {if(!(expression)) {InvalidCodePath;}}
#else
#define Assert(expression)
#endif

#define ArrayCount(array) (sizeof(array)/sizeof(array[0]))

#define TRUE 1
#define FALSE 0

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef uint32_t b32;

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;

typedef float f32;
typedef double f64;

#define U8_MAX 0xffff
#define U32_MAX 0xffffffffffffffff

struct grid
{
    u8 *Data;
    u32 Width;
    u32 Height;
};

struct heap_node
{
    u32 HeapIndex;
    f32 Cost;
    //Below is relevant to Pathfinding code
    u32 GridIndex;
    u32 ParentIndex;
    f32 GCost;
    u16 OpenID;
    u16 CloseID;
    //below is used for preprocessing algorithms (Subgoal)
    u8 Clearances[8];
    u32* Connected;
    u32 ConnectedCapacity;
    u32 ConnectedSize;
};

struct u32_pair 
{
    u32 X, Y;
};

struct f32_pair
{
    float X, Y;
};

f32 RayBoxTest(f32 RayX, f32 RayY, f32 DirX, f32 DirY, f32 MinX, f32 MinY, f32 MaxX, f32 MaxY, f32 DefaultValue);

#endif
