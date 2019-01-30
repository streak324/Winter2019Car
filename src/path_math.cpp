#ifndef SUMMER_MATH_CPP
#define SUMMER_MATH_CPP

#include <math.h>
#include "path_common.h"
#include "path_math.h"
#define SIN_BITS 14
#define SIN_MASK (~(-1 << SIN_BITS))
#define SIN_COUNT (SIN_MASK+1)
#define RAD_TO_INDEX (((f32)SIN_COUNT) / (2*PI32))
#define INDEX_TO_RAD (2 * PI32 / SIN_COUNT)
#define EPSILON 1.0f / 1024.0f
#define IsWithinTolerance(value, desired) ((value - desired >= 0 && value - desired <= EPSILON) || (value - desired <= 0 && value - desired >= -EPSILON))
#define IsAboveTolerance(value, desired) ((value - desired) >= -EPSILON)
#define IsBelowTolerance(value, desired) ((value - desired) <= EPSILON)

f32 SinLookup[SIN_COUNT];
void PrecomputeTrig()
{
    for(u32 Count=0; Count < SIN_COUNT; ++Count)
    {
        f32 Rads = 2 * PI32 * ((Count + 0.5f) / SIN_COUNT);
        f32 Multiplier = 1.0f;
        if(Count >= SIN_COUNT/2)
        {
            Rads = Rads - PI32;
            Multiplier = -1;
        }
        SinLookup[Count] = Multiplier * (16 * Rads*(PI32 - Rads)) / (5*PI32*PI32 - 4*Rads*(PI32 - Rads));
        if(Count == 0)
            SinLookup[Count] = 0;
    }
    SinLookup[0] = 0;
    SinLookup[SIN_COUNT >> 1] = 0;
}

u64 Xorshift64(random64* Rand)
{
    u64 NewState = Rand->State;
    NewState ^= NewState >> 12;
    NewState ^= NewState << 25;
    NewState ^= NewState >> 27;
    Rand->State = NewState;
    return NewState * 0x2545F4914F6CDD1D;
}

f32 RandomF32Unit(random64* Rand)
{
    u64 Value = Xorshift64(Rand);
    f32 Result = 1 - 2 * (((f32) Value) / MAX_U64);
    return Result;
}

f32 Sqrt(f32 Value)
{
    return sqrtf(Value);
}

f32 DegToRad(f32 Deg)
{
    return Deg * (PI32 / 180.0f);
}

f32 Cos(f32 Rads)
{
#if 1
    return cosf(Rads);
#else
    return Sin(PI32/ - Rads);
#endif
}

f32 Acos(f32 Rads)
{
    return acosf(Rads);
}

f32 Sin(f32 Rads)
{
#if 1
    return sinf(Rads);
#else
    return SinLookup[((u32) (Rads * RAD_TO_INDEX)) & SIN_MASK];
#endif
}

f32 Tan(f32 Rads)
{
    return Sin(Rads) / Cos(Rads);
}

f32 SquaredLen(v2 Vec)
{
    return Vec.X*Vec.X + Vec.Y*Vec.Y;
}

f32 Len(f32 X, f32 Y)
{
    return Sqrt(X*X + Y*Y);
}

f32 Len(f32 X, f32 Y, f32 Z)
{
    return Sqrt(X*X + Y*Y + Z*Z);
}

f32 Len(v3 V)
{
    return Len(V.X, V.Y, V.Z);
}


f32 Min(f32 A, f32 B)
{
    return A < B ? A : B;
}

f32 Max(f32 A, f32 B)
{
    return A > B ? A : B;
}


v2 operator+(v2 A, v2 B)
{
    return {A.X + B.X, A.Y + B.Y};
}

v2 operator-(v2 A, v2 B)
{

    return {A.X - B.X, A.Y - B.Y};
}

v3 operator+(v3 V1, v3 V2)
{
    return
    {
        V1.X + V2.X,
        V1.Y + V2.Y,
        V1.Z + V2.Z,
    };
}

v3 operator-(v3 V)
{
    return
    {
        -V.X,
        -V.Y,
        -V.Z
    };
}

v3 operator-(v3 A, v3 B)
{
    return
    {
        A.X - B.X,
        A.Y - B.Y,
        A.Z - B.Z
    };
}


v3 operator*(mat4 Mat, v3 V3)
{
    return
    {
        Mat.M00 * V3.X + Mat.M10 * V3.Y + Mat.M20 * V3.Z + Mat.M30 * 1,
        Mat.M01 * V3.X + Mat.M11 * V3.Y + Mat.M21 * V3.Z + Mat.M31 * 1,
        Mat.M02 * V3.X + Mat.M12 * V3.Y + Mat.M22 * V3.Z + Mat.M32 * 1,
    };
}

v3 operator*(f32 Scale, v3 V)
{
    return
    {
        V.X * Scale,
        V.Y * Scale,
        V.Z * Scale,
    };
}


v3 operator*(v3 V, f32 Scale)
{
    return
    {
        V.X * Scale,
        V.Y * Scale,
        V.Z * Scale,
    };
}

v3 operator*(v3 Vec1, v3 Vec2)
{
    return {Vec1.X * Vec2.X, Vec1.Y * Vec2.Y, Vec1.Z * Vec2.Z};
}

v4 operator*(mat4 Mat, v4 V4)
{
    return
    {
        Mat.M00 * V4.X + Mat.M10 * V4.Y + Mat.M20 * V4.Z + Mat.M30 * V4.W,
        Mat.M01 * V4.X + Mat.M11 * V4.Y + Mat.M21 * V4.Z + Mat.M31 * V4.W,
        Mat.M02 * V4.X + Mat.M12 * V4.Y + Mat.M22 * V4.Z + Mat.M32 * V4.W,
        Mat.M03 * V4.X + Mat.M13 * V4.Y + Mat.M23 * V4.Z + Mat.M33 * V4.W
    };
}

quat operator*(quat A, quat B)
{
    v3 ResultAxis = Cross(A.Axis, B.Axis) + A.W * B.Axis + B.W*A.Axis;
    float ResultW = A.W*B.W - Dot(A.Axis, B.Axis);

    quat Result = {ResultAxis, ResultW};
    return Result;
}

quat operator+(quat Q1, quat Q2)
{
    return 
    {
        Q1.X + Q2.X,
        Q1.Y + Q2.Y,
        Q1.Z + Q2.Z,
        Q1.W + Q2.W,
    };
}

mat4 operator*(mat4 M1, mat4 M2)
{
    mat4 Result = {};
    for(u32 Row=0; Row < 4; ++Row)
    {
        for(u32 Col=0; Col < 4; ++Col)
        {
            Result.M[Row*4 + Col] = 
            M1.M[Row*4] * M2.M[Col] + M1.M[Row*4 + 1] * M2.M[4 + Col] + 
            M1.M[Row*4 + 2] * M2.M[8 + Col] + M1.M[Row*4 + 3] * M2.M[12 + Col];
        }
    }
    return Result;
}

mat4 CreateIdentity()
{
    return {
        1, 0, 0, 0, 
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };
}

mat4 PerspectiveMatrix(f32 ViewAngle, f32 Aspect, f32 Near, f32 Far)
{
    f32 TanView = Tan(ViewAngle);
    f32 Top = Near * TanView;
    f32 Right = Top * Aspect;

    mat4 Persp = {};
    Persp.M00 = Near / Right;
    Persp.M11 = Near / Top;
    Persp.M22 = -(Far + Near) / (Far - Near);
    Persp.M23 = -(2 * Far * Near) / (Far - Near);
    Persp.M32 = -1;
    return Persp;
}

mat4 OrthographicMatrix(f32 left, f32 right, f32 bottom, f32 top, f32 NearView, f32 FarView)
{
	mat4 Matrix = {};
	Matrix.M00 = 2/(right - left);
	Matrix.M03 = -(right + left)/(right - left);
	Matrix.M11 = 2/(top - bottom);
    Matrix.M13 = - (top + bottom) / (top - bottom);
	Matrix.M22 = -2/(FarView - NearView);
    Matrix.M23 = - (FarView + NearView) / (FarView - NearView);
    Matrix.M33 = 1;
    return Matrix;
}

mat4 ScaleMatrix(f32 X, f32 Y, f32 Z)
{
    mat4 Iden = CreateIdentity();
    Iden.M00 = X;
    Iden.M11 = Y;
    Iden.M22 = Z;
    return Iden;
}

mat4 ScaleMatrix(v3 Volume)
{
    return ScaleMatrix(Volume.X, Volume.Y, Volume.Z);
}

mat4 CreateTranslation(f32 X, f32 Y, f32 Z)
{
    mat4 Mat = {};
    Mat.M00 = 1;
    Mat.M03 = X;
    Mat.M11 = 1;
    Mat.M13 = Y;
    Mat.M22 = 1;
    Mat.M23 = Z;
    Mat.M33 = 1;
    return Mat;
}

mat4 CreateTranslation(v3 V3)
{
    return CreateTranslation(V3.X, V3.Y, V3.Z);
}

//Note(andrew): This function assumes that the quaternion is normalized
mat4 QuatToMat4(quat Q)
{
    f32 XSq = Q.X * Q.X;
    f32 YSq = Q.Y * Q.Y;
    f32 ZSq = Q.Z * Q.Z;
    f32 WSq = Q.W * Q.W;
    f32 TwoX = 2 * Q.X;
    f32 TwoY = 2 * Q.Y;
    f32 TwoW = 2 * Q.W;
    f32 XY = TwoX * Q.Y;
    f32 XZ = TwoX * Q.Z;
    f32 YZ = TwoY * Q.Z;
    f32 WX = TwoW * Q.X;
    f32 WY = TwoW * Q.Y;
    f32 WZ = TwoW * Q.Z;

    mat4 Result = {};
    Result.M00 = WSq + XSq - YSq - ZSq;
    Result.M01 = XY - WZ;
    Result.M02 = XZ + WY;
    Result.M10 = XY + WZ;
    Result.M11 = WSq - XSq + YSq - ZSq;
    Result.M12 = YZ - WX;
    Result.M20 = XZ - WY;
    Result.M21 = YZ + WX;
    Result.M22 = WSq - XSq - YSq + ZSq;
    Result.M33 = 1;
    return Result;
}

mat4 CreateView(v3 Right, v3 Up, v3 Forward, v3 Pos)
{
    mat4 Result = {
        Right.X, Right.Y, Right.Z, -Dot(Right, Pos),
        Up.X, Up.Y, Up.Z, -Dot(Up, Pos),
        Forward.X, Forward.Y, Forward.Z, -Dot(Forward, Pos),
        0, 0, 0, 1
    };
    return Result;
}

mat4 CreateView(quat Rotation, v3 Pos)
{
    return QuatToMat4(Rotation) * CreateTranslation(-Pos);
}

void RotateV3(v3* V, quat Q)
{
    quat VectorQuat = {V->X, V->Y, V->Z, 1.0f};
    quat ResultQuat = Q * VectorQuat * Conjugate(Q);

    V->X = ResultQuat.X;
    V->Y = ResultQuat.Y;
    V->Z = ResultQuat.Z;
}

v3 RotateV3(v3 V, quat Q)
{
    quat VectorQuat = {V.X, V.Y, V.Z, 1.0f};
    quat ResultQuat = Q * VectorQuat * Conjugate(Q);
    return
    {
        ResultQuat.X,
        ResultQuat.Y,
        ResultQuat.Z
    };
}

void RotateV3(v3* V, v3 Axis, f32 Angle)
{
    quat Quat = CreateRotation(Axis, Angle);
    RotateV3(V, Quat);
}

v3 RotateV3(v3 V, v3 Axis, f32 Angle)
{
    quat Quat = CreateRotation(Axis, Angle);
    return RotateV3(V, Quat);
}

v3 RotateAroundPoint(v3 RotatingPoint, v3 CenterPoint, quat Orientation)
{
   v3 RelativeRotation = RotateV3(RotatingPoint - CenterPoint, Orientation);
   return RelativeRotation + CenterPoint;
}


v3 Cross(v3 A, v3 B)
{
   return
   {
        A.Y * B.Z - A.Z * B.Y,
        A.Z * B.X - A.X * B.Z,
        A.X * B.Y - A.Y * B.X
   };
}


v3 ChangeMagnitude(v3 V, f32 NewLen)
{
    f32 OldLen = Len(V);
    f32 LenRatio = NewLen / OldLen;
    return
    {
        V.X = LenRatio,
        V.Y = LenRatio,
        V.Z = LenRatio
    };
}

quat CreateRotation(f32 X, f32 Y, f32 Z, f32 Theta)
{
    f32 SinAngle = Sin(Theta/2);
    quat Quat = 
    {
        X * SinAngle,
        Y * SinAngle,
        Z * SinAngle,
        Cos(Theta/2)
    };
    NormQuat(&Quat);
    return Quat;
}

quat CreateRotation(v3 V, f32 Theta)
{
    return CreateRotation(V.X, V.Y, V.Z, Theta);
}

quat CreateRotation(v3 From, v3 To)
{
    f32 Angle = GetAngle(From, To);
    if(IsWithinTolerance(Angle, 0))
    {
        return IdentityQuat;
    }
    v3 RotationAxis = NormV3(Cross(From, To));
    return CreateRotation(RotationAxis, Angle);
}

f32 Dot(v3 A, v3 B)
{
    return A.X * B.X + A.Y * B.Y + A.Z * B.Z;
}

f32 Dot(quat Q1, quat Q2)
{
    return Q1.X * Q2.X + Q1.Y * Q2.Y + Q1.Z * Q2.Z + Q1.W * Q2.W;
}

f32 GetAngle(v3 A, v3 B)
{
    f32 DotProd = Dot(A, B);
    f32 LenA = Len(A);
    f32 LenB = Len(B);
    if(IsWithinTolerance(Len(A), 0.0f) || IsWithinTolerance(Len(B), 0.0f))
    {
        return 0.0f;
    }
    Assert(Len(A) != 0);
    Assert(Len(B) != 0);
    Assert(IsWithinTolerance(LenA, 1.0f) || IsWithinTolerance(LenB, 1.0f));
    f32 Value = DotProd / (LenA * LenB);
    Assert(IsBelowTolerance(Value, 1.0f) || IsAboveTolerance(Value, -1.0f));
    Value = Value > 1.0f ? 1.0f : Value;
    Value = Value < -1.0f ? -1.0f : Value;
    f32 Angle = Acos(Value);
    return Angle;
}

quat NLerp(quat Q1, quat Q2, f32 Weight)
{
    quat Result = {};
    Result.X = (1 - Weight) * Q1.X + Q2.X * Weight;
    Result.Y = (1 - Weight) * Q1.Y + Q2.Y * Weight;
    Result.Z = (1 - Weight) * Q1.Z + Q2.Z * Weight;
    Result.W = (1 - Weight) * Q1.W + Q2.W * Weight;
    NormQuat(&Result);
    return Result;
}

quat Conjugate(quat Q)
{
    return  {-Q.X, -Q.Y, -Q.Z, Q.W};
}

quat LookAt(v3 Forward, v3 Up)
{
    return IdentityQuat;
}

void NormQuat(quat *Q)
{
    f32 Len = Sqrt(Dot(*Q, *Q));
    Q->X = Q->X / Len;
    Q->Y = Q->Y / Len;
    Q->Z = Q->Z / Len;
    Q->W =  Q->W / Len;
}

void NormV3(v3 *V3)
{
    f32 Dist = Len(V3->X, V3->Y, V3->Z);
    if(Dist > 0)
    {
        V3->X = V3->X / Dist;
        V3->Y = V3->Y / Dist;
        V3->Z = V3->Z / Dist;
    }
}

v3 NormV3(v3 A)
{
    f32 Dist = Len(A.X, A.Y, A.Z);
    if(Dist > 0)
    {
        A.X = A.X / Dist;
        A.Y = A.Y / Dist;
        A.Z = A.Z / Dist;
    }
    return A;
}


void ApplyScale(mat4 *Mat, f32 Scale)
{
    for(u32 i=0; i < 16; ++i)
    {
        Mat->M[i] *= Scale;
    }
}

void ApplyScale(v3* V, f32 scale)
{
    V->X *= scale;
    V->Y *= scale;
    V->Z *= scale;
}

b32 IsEqual(v3 One, v3 Two, f32 Epsilon)
{
    return 
            One.X <= Two.X + EPSILON && One.X + EPSILON >= Two.X &&
            One.Y <= Two.Y + EPSILON && One.Y + EPSILON >= Two.Y &&
            One.Z <= Two.Z + EPSILON && One.Z + EPSILON >= Two.Z;
}

#endif
