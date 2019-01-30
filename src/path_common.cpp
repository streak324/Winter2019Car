#include "path_common.h"


f32 RayBoxTest(f32 RayX, f32 RayY, f32 DirX, f32 DirY, f32 MinX, f32 MinY, f32 MaxX, f32 MaxY, f32 DefaultValue)
{
    f32 t1 = (MinX - RayX)/DirX;
    f32 t2 = (MaxX - RayX)/DirX;
    f32 t3 = (MinY - RayY)/DirY;
    f32 t4 = (MaxY - RayY)/DirY;
    f32 tmin = fmax(fmin(t1, t2), fmin(t3, t4));
    f32 tmax = fmin(fmax(t1, t2), fmax(t3, t4));
    return (tmax < 0 || tmin > tmax) ? DefaultValue : tmin;
}

