#ifndef SUMMER_MATH_H
#define SUMMER_MATH_H

#define PI32  3.1415927f
#define TAU32 2.0f*PI32
#include "path_common.h"
#define MAX_U64 0xFFFFFFFFFFFFFFFF 

struct v2
{
    f32 X, Y;
};

union v3
{
    f32 F[3];
    struct
    {
        float X, Y, Z;
    };
};

struct quat
{
    union
    {
        v3 Axis;
        struct
        {
            f32 X, Y, Z;
        };
    };
    f32 W;
};

union v4
{
    f32 F[4];
    struct
    {
        f32 X, Y, Z, W;
    };
};

//Note(andrew): colum-major order
union mat4
{
    f32 M[16];
    struct
    {
        f32 M00, M01, M02, M03,
            M10, M11, M12, M13,
            M20, M21, M22, M23,
            M30, M31, M32, M33;
    };
};

struct random64
{
    u64 State;
};

const v3 XAxis = {1, 0, 0};
const v3 YAxis = {0, 1, 0};
const v3 ZAxis = {0, 0, 1};
const quat IdentityQuat = {0, 0, 0, 1};

void PrecomputeTrig();
u64 Xorshift64(random64* Rand);
f32 RandomF32Unit(random64* Rand);
f32 Sqrt(f32 Value);
f32 Cos(f32 rads);
f32 Acos(f32 rads);
f32 Sin(f32 rads);
f32 Tan(f32 rads);
f32 SquaredLen(v2 Vec);
f32 Len(f32 X, f32 Y);
f32 Len(f32 X, f32 Z, f32 Y);
f32 Len(v3 V);
f32 Min(f32 A, f32 B);
f32 Max(f32 A, f32 B);
v2 operator+(v2 A, v2 B);
v2 operator-(v2 A, v2 B);
v3 operator+(v3 V1, v3 V2);
v3 operator-(v3 V1);
v3 operator-(v3 A, v3 B);
v3 operator*(mat4 Mat, v3 V3);
v3 operator*(f32 Scale, v3 V3);
v3 operator*(v3 V3, f32 Scale);
v3 operator*(v3 Vec1, v3 Vec2);
v4 operator*(mat4 Mat, v4 V4);
quat operator*(quat Q1, quat Q2);
mat4 operator*(mat4 M1, mat4 M2);
mat4 CreateIdentity();
mat4 PerspectiveMatrix(f32 ViewAngle, f32 Aspect, f32 Near, f32 Far);
mat4 OrthographicMatrix(f32 left, f32 right, f32 bottom, f32 top, f32 NearView, f32 FarView);
mat4 ScaleMatrix(f32 X, f32 Y, f32 Z);
mat4 ScaleMatrix(v3 Volume);
mat4 CreateTranslation(f32 X, f32 Y, f32 Z);
mat4 CreateTranslation(v3 V3);
mat4 QuatToMat4(quat Q);
mat4 CreateView(v3 Right, v3 Up, v3 Forward, v3 Pos);
mat4 CreateView(quat Rotation, v3 Pos);
void RotateV3(v3* V3, quat Q);
v3 RotateV3(v3 V, quat Q);
void RotateV3(v3* V3, v3 Axis, f32 Angle);
v3 RotateV3(v3 V3, v3 Axis, f32 Angle);
v3 RotateAroundPoint(v3 RotatingPoint, v3 CenterPoint, quat Orientation);
inline v3 Cross(v3 V1, v3 V2);
v3 ChangeMagnitude(v3 V, f32 NewLen);
f32 Dot(v3 V1, v3 V2);
f32 Dot(quat Q1, quat Q2);
f32 GetAngle(v3 A, v3 B);
f32 GetAngle(f32 Angle1, f32 Dist1, f32 Angle2, f32 Dist2);
quat CreateRotation(f32 X, f32 Y, f32 Z, f32 Theta);
quat CreateRotation(v3 V, f32 Theta);
quat CreateRotation(v3 From, v3 To);
quat NLerp(quat Q1, quat Q2, f32 Weight);
quat Conjugate(quat Q1);
quat LookAt(v3 RelativePos, v3 Up);
void NormQuat(quat* Q);
void NormV3(v3 *V3);
v3 NormV3(v3 V3);
void ApplyScale(mat4 *Mat, f32 scale);
void ApplyScale(v3* V, f32 scale);
b32 IsEqual(v3 One, v3 Two, f32 Epsilon);
#endif
