#include "path_common.h"
#include "path_math.h"
#include "path_car.h"
#include "path_car_ai.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "race/drive_param.h"
#include "sensor_msgs/LaserScan.h"
#include <stdio.h>
#include <vector>

ros::Publisher DrivePublisher;
ros::Subscriber ScanSubscriber;
ros::Subscriber KillSubscriber;
ros::Subscriber UISubscriber;
driver Driver;
car Car;
f32 StandardMotorSpeed;
vfh_state VFH;
vph_state VPH;
follow_gap FTG;
b32 UseDriver;
f32 VFHUnitsPerMeter;
f32 MaxScanRange;
race::drive_param DriveParams;

char* UserInterfaceTopic = "algo_selection";

//TODO(andrew): Not sure, but we may have race conditions when modifying the same data...
//across different callbacks that are running simultaneously.
//We should add some mutex to the vulnerable code

void ProcessUserInput(const std_msgs::Int64::ConstPtr& UserInput)
{
    i64 Switch = UserInput->data;
    switch(Switch)
    {
        case 0: 
            Driver.AvoidanceMethod = Avoid_FTG;
            break;
        case 1: 
            Driver.AvoidanceMethod = Avoid_VFH;
            break;
        case 2: 
            Driver.AvoidanceMethod = Avoid_VPH;
            break;
        default:
            ROS_INFO("HOLD IT THERE PARDNER. You gave me an integer value that I don't know what to do with.\n");
            break;
    }
}

void KillDriver(const std_msgs::Bool::ConstPtr& KillVFH)
{
    ROS_DEBUG("Killing Driver\n");
    DriveParams.velocity = 0;
    DriveParams.angle = 0;
    DrivePublisher.publish(DriveParams);
    UseDriver = false;
}

void CollectScanDataAndRunVFH(const sensor_msgs::LaserScan::ConstPtr& Scan)
{
    for(i32 Count=180; Count < 900; Count += 4)
    {
        if(Scan->range_min > Scan->ranges[Count] && Scan->range_max < Scan->ranges[Count])
        {
            Car.ScanDistances[(Count - 180)/4] = Scan->range_min;
        }
        else
        {
            Car.ScanDistances[(Count - 180)/4] = Scan->ranges[Count];
        }
    }
    if(UseDriver)
    {
        RunDriver(&Driver, &Car, &VFH, &VPH, &FTG);
        DriveParams.velocity = Car.MotorSpeed;
        ROS_INFO("VELOCITY: %f\n", FTG.SpeedOutput);
        DriveParams.angle = -100.0f * Car.TurnAngle / Car.MaxTurnAngle + 8.0f;
        DrivePublisher.publish(DriveParams);
    }
    else
    {
        DriveParams.velocity = 0;
        DriveParams.angle = 0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "driver");
    ros::NodeHandle Node;

    ROS_INFO("VFH Driver LOCKED AND LOADED\n");

    DrivePublisher = Node.advertise<race::drive_param>("drive_parameters", 100);
    ScanSubscriber = Node.subscribe<sensor_msgs::LaserScan>("/scan", 10, CollectScanDataAndRunVFH);
    KillSubscriber = Node.subscribe<std_msgs::Bool>("eStop", 10, KillDriver);
    UISubscriber = Node.subscribe<std_msgs::Int64>(UserInterfaceTopic, 10, ProcessUserInput);

    DriveParams.velocity = 0;
    DriveParams.angle = 0;
    DrivePublisher.publish(DriveParams);

    Car.Angle = 0;
    Car.Width = 0.25f;
    Car.Height = 0.5f;
    Car.MinSensorAngle = -PI32/2;
    Car.MaxSensorAngle = PI32/2;
    Car.MaxTurnAngle = PI32/4.0f;
    Car.MaxSensorRange = 4.0f;
    Car.ScanSize = 180;
    Car.ScanDistances = (f32*) malloc(sizeof(f32) * Car.ScanSize);
    memset(Car.ScanDistances, 0, sizeof(f32) * Car.ScanSize);
    VFHUnitsPerMeter = 10.0f;

    Driver.MinVFHThreshold = 0.4f;
    Driver.MaxVFHThreshold = 0.7f;
    Driver.SpeedOptions = (f32*) malloc(3 * sizeof(f32));
    Driver.SpeedOptions[0] = 20.0f;
    Driver.SpeedOptions[1] = 17.5f;
    Driver.SpeedOptions[2] = 15.0f;
    Driver.NumberOfOptions = 3;

    Driver.AvoidanceMethod = Avoid_FTG;
    UseDriver = true;

    InitFTG(&FTG, &Car);
    FTG.Threshold = 0.4f;
    FTG.scan_dist = 2.0f;
    FTG.speed_factor = 0.4f;
    FTG.MinGapSize = 20;
    FTG.StopThreshold = Car.Height * 2;
    FTG.SafetyDistance = 0.5f * Car.Height;
    FTG.MinSpeed = 17.5f;
    FTG.MaxSpeed = 20.0f;

    f32 SafetyRadius = 0.1f;
    i32 VFHSectorSize = 180;
    f32 CarRadius = sqrtf(Car.Width*Car.Width/4 + Car.Height*Car.Height/4);
    InitVFH(&VFH, CarRadius * VFHUnitsPerMeter, (i32) (2 * VFHUnitsPerMeter * Car.MaxSensorRange), VFHSectorSize, SafetyRadius * VFHUnitsPerMeter, 0);
    VFH.MinSpeed = 15.0f;
    VFH.MaxSpeed = 20.0f;
    VFH.FollowTarget = false;
    VFH.Threshold = 0.4f;
    VFH.TargetDirectionWeight = 5;
    VFH.CurrentDirectionWeight = 2;
    VFH.PreviousDirectionWeight = 2;

    InitVPH(&VPH, &Car);
    VPH.DistanceThreshold = CarRadius * 1.25f;
    VPH.MaxSpeed = 20.0f;
    VPH.SafetyCoeff = 2.0f;
    VPH.Accel = 10.0f;

    ros::spin();
    return 0;
}
