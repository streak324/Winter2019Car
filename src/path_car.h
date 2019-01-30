#ifndef PATH_CAR_H
#define PATH_CAR_H

#include "path_common.h"

//only used in the simulation code. Not used in the actual car
struct car
{
    f32 CenterX;
    f32 CenterY;
    f32 Width;
    f32 Height;
    f32 Velocity;
    f32 MotorSpeed; 
    f32 TurnAngle;//<0 goes right, >0 goes left
    f32 Angle;
    f32 MaxSensorRange; //domain: (0, PI)
    f32 MinSensorAngle; //domain: (-PI, 0)
    f32 MaxSensorAngle;
    f32 MaxTurnAngle;
    //distances found from the sensor. begins from MinSensorAngle to MaxSensorAngle relative to car's direction
    f32 *ScanDistances;
    i32 ScanSize;
    f32 WheelFriction;
    f32 Mass;
};

f32 GetScanAnglePrecision(f32 MinScanAngle, f32 MaxScanAngle, i32 ScanSize);
f32 ApproximateEffectiveTurnRadius(f32 Speed, f32 TurnAngle, f32 Wheelbase, f32 WheelFriction);
f32 ComputeTurnRadius(f32 Wheelbase, f32 TurnAngle);
f32 GetAngleOfScanSample(i32 SampleIndex, i32 ScanSize, f32 MinSensorAngle, f32 MaxSensorAngle);
f32 GetCarRadius(car* Car);
void SimulateScanSensor(car* Car, grid Grid);
void ChangeTurnAngle(car* Car, f32 NewTurnAngle);
void UpdateCar(car* Car, grid Grid, f32 UpdateTime);
#endif
