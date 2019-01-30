#include "path_car.h"
#include "math.h"

#define PI32 3.14159267f

//TODO(andrew): Figure out a way to approximate EFFECTIVE turn radius
f32 ApproximateEffectiveTurnRadius(f32 Speed, f32 TurnAngle, f32 Wheelbase, f32 WheelFriction)
{
    return Wheelbase / tanf(TurnAngle);
}

f32 GetScanAnglePrecision(f32 MinScanAngle, f32 MaxScanAngle, i32 ScanSize)
{
    Assert(MinScanAngle < MaxScanAngle);
    return (MaxScanAngle - MinScanAngle) / (f32) ScanSize;
}

f32 GetCarRadius(car* Car)
{
    return sqrtf(Car->Width * Car->Width/4 + Car->Height * Car->Height / 4);
}

//This computed turn radius wont be practical with real world high velocity grounded vehicles
//because it doesn't consider traction
f32 ComputeTurnRadius(f32 Wheelbase, f32 TurnAngle)
{
    return Wheelbase / tanf(TurnAngle);
}

f32 GetAngleOfScanSample(i32 ScanIndex, i32 ScanSize, f32 MinSensorAngle, f32 MaxSensorAngle)
{
    Assert(MaxSensorAngle > MinSensorAngle);
    f32 TotalSensorCoverage = MaxSensorAngle - MinSensorAngle;
    return ((f32)ScanIndex/(f32) ScanSize * TotalSensorCoverage + MinSensorAngle);
}

void SimulateScanSensor(car* Car, grid Grid)
{
    for(u32 Count=0; Count < Car->ScanSize; ++Count)
    {
        Car->ScanDistances[Count] = Car->MaxSensorRange;
        f32 RangeAngle = Car->Angle + GetAngleOfScanSample(Count, Car->ScanSize, Car->MinSensorAngle, Car->MaxSensorAngle);
        f32 RangeDirX = cosf(RangeAngle);
        f32 RangeDirY = sinf(RangeAngle);

        i32 IntMaxRange = (i32) ceilf(Car->MaxSensorRange);
        i32 IntCarX = (i32) floorf(Car->CenterX);
        i32 IntCarY = (i32) floorf(Car->CenterY);
        for(i32 IntGridY = IntCarY - IntMaxRange; IntGridY < IntCarY + IntMaxRange; ++IntGridY)
        {
            if(IntGridY < 0 || IntGridY >= Grid.Height) continue;

            for(i32 IntGridX = IntCarX - IntMaxRange; IntGridX < IntCarX + IntMaxRange; ++IntGridX)
            {
                if(IntGridX < 0 || IntGridX >= Grid.Width) continue;
                if(Grid.Data[IntGridX + IntGridY * Grid.Width] == BLOCKED_CELL)
                {
                    f32 GridX = (f32) IntGridX;
                    f32 GridY = (f32) IntGridY;
                    f32 MinX = GridX - 0.5f;
                    f32 MinY = GridY - 0.5f;
                    f32 MaxX = GridX + 0.5f;
                    f32 MaxY = GridY + 0.5f;
                    f32 NewRange = RayBoxTest(Car->CenterX, Car->CenterY, RangeDirX, RangeDirY, MinX, MinY, MaxX, MaxY, Car->MaxSensorRange);
                    if(NewRange < Car->ScanDistances[Count])
                    {
                       Car->ScanDistances[Count] = NewRange;
                    }
                    if(NewRange < 0)
                    {
                        Car->ScanDistances[Count] = 0;
                    }
                }
            }
        }
    }
}

void ChangeTurnAngle(car* Car, f32 NewTurnAngle)
{
    Car->TurnAngle = NewTurnAngle;
    if(Car->TurnAngle > Car->MaxTurnAngle)
    {
        Car->TurnAngle = Car->MaxTurnAngle;
    }
    else if(Car->TurnAngle < -Car->MaxTurnAngle)
    {
        Car->TurnAngle = -Car->MaxTurnAngle;
    }
}

void UpdateCar(car* Car, grid Grid, f32 DeltaTime)
{
    if(Car->TurnAngle > Car->MaxTurnAngle)
    {
        Car->TurnAngle = Car->MaxTurnAngle;
    }
    else if(Car->TurnAngle < -Car->MaxTurnAngle)
    {
        Car->TurnAngle = -Car->MaxTurnAngle;
    }

    f32 TurnRadius = Car->Height / tanf(Car->TurnAngle);
    f32 DeltaVelocity = Car->MotorSpeed - Car->Velocity;
    Car->Velocity += DeltaVelocity;

    f32 Accel = Car->Velocity * Car->Velocity / TurnRadius;
    if((Car->TurnAngle > 0.01f || Car->TurnAngle < -0.01f))
    {
        f32 DeltaAngle = (DeltaTime * Car->Velocity) / TurnRadius;
        Car->Angle += DeltaAngle;
    }
    Car->CenterX = Car->CenterX + Car->Velocity * DeltaTime * cosf(Car->Angle);
    Car->CenterY = Car->CenterY + Car->Velocity * DeltaTime * sinf(Car->Angle);
}

