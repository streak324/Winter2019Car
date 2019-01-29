#ifndef PATH_CAR_AI_H
#define PATH_CAR_AI_H

#include "path_car.h"

struct vfh_state
{
    //if this is changed, must resize the tables and recompute (RegionDiamter x RegionDiameter) tables{
    //this should not be bigger than the range of the sensor data
    i32 MaxRegionDiameter;
    i32 RegionDiameter;
    f32 CellUnitsToWorld;

    i32 AngularSectors;//if this is changed, must resize the histogram arrays
    f32 EnlargedRadius; //must call 

    f32 Threshold; //tunable. default inside InitVFH
    i32 MinGapSize; //tunable. default inside InitVFH

    i32 TargetDirectionWeight; //tunable. default is 5
    i32 CurrentDirectionWeight; //tunable. default is 2
    i32 PreviousDirectionWeight; //tunable. default is 2

    f32 MinSpeed; //tunable
    f32 MaxSpeed; //tunable

    b32 FollowTarget;//if true, VFH wont be following a goal and will randomly explore the environment

    //VFH uses this as a guide when trying to follow a target
    f32 TargetHeading;//should be relative to the car and updated when the car's position/direction changes

    //These are tables of RegionDiameter by RegionDiameter in size
    i8* Region;//relative to the car position 
    f32* RegionInfluence;//Entries should not be changed
    f32* RegionDistances;//Entries should not be changed
    f32* RegionDirections;//Entries should not be changed
    f32* EnlargedAngles; //Entries should not be changed UNLESS the enlarged radius is changed

    i32* SectorBins; //The tiles in the region that would overlap or be within a Sector
    i32* SectorBinSizes;
    i32 BinsSize;
    i32 BinsCapacity;

    //These arrays should be AngularSectors in size and are used for intermediary calculations.
    //Dont try to modify them
    f32* PrimaryHistogram;
    b32* BinaryHistogram;
    b32* MaskedHistogram;
    i32* SectorGaps;
    i32* CandidateSectors;
    i32* CandidateCosts;

    b32 GapExists;//dont change this. this is output
    i32 NumberOfCandidates;//dont change this. this is output
    f32 SelectedAngle;//dont change this. this is output. The angle is relative to the car
    i32 SelectedSector;//dont change this. this is output
    i32 TargetSector;//dont change this. this is output
    i32 CurrentSector;//dont change this. this is output
    f32 SpeedOutput;//output
};

enum avoidance_method
{
    Avoid_VFH = 0,
    Avoid_VPH = 1,
    Avoid_FTG = 2,
};

struct vph_state
{
    f32 DistanceThreshold;//tunable. must be larger than the car radius
    f32 MaxSpeed;//tunable
    f32 SafetyCoeff;//tunable (suggested 1.2 - 1.5)
    f32 Accel;//tunable

    f32 Interval;//Should be the inverse of the rate at which you call RunVPH

    f32* ModifiedDistances;//must be same size as the scan array
    i32* GroupStartIndices;//must be the same size as the scan array
    i32* GroupID;//must be the same size as the scan array
    b32* IsConcave;//must be the same size as the scan array
    b32* Thresholds;//must be the same size as the scan array
    i32 NumberOfGroups;

    f32 TurnAngleOutput;//output
    f32 SpeedOutput;//output
};

struct follow_gap
{
	// Variables for Follow the Gap Method
	i32 max_gapWidth; //= -1
	i32 max_gap_start; //= -1
	i32 max_gap_end; //= -1
	f32 max_gapCenter; //= -1
	i32 center_gap_index; //= -1
	f32* Obstacles; // holds distance values to obstacles
    i32 NumberOfObstacles;
	i32 *lidar_gap;	// lidar_gap = [10.0] * (180 * self.car.reading_number + 1)
	i32 lidar_gap_size;
    i32 MinGapSize;//tunable: 20 suggested
	f32 Threshold; //tunable: 0.4
	f32 scan_dist; //tunable: 2
    f32 StopThreshold;//tunable: Car->Height * 2
    f32 SafetyDistance;//tunable: half the car height
	f32 speed_factor; //tunable: 0.4 car's speed limit: range 0.2 (min) to 1
	//self.goal_pos = [self.car._position[0], self.car._position[1]]
	//goal_dist = 0
    f32 MinSpeed;//tunable
    f32 MaxSpeed;//tunable
    f32 TurnAngleOutput;
    f32 SpeedOutput;
};

struct gapStruct
{
	i32 gapStart;
	i32 gapEnd;
	i32 gapWidth;
};

struct driver 
{
    avoidance_method AvoidanceMethod;
#if 1
    f32 MinVFHThreshold;//tunable for vfh
    f32 MaxVFHThreshold;//tunable for vfh
    f32* SpeedOptions;//tunable. array should be from biggest to smallest
    i32 NumberOfOptions;//should be the array size SpeedOptions
    i32 SelectedOption;//output
#endif
    b32 StopCar;//output
};

f32 GetMaxVFHPrimaryValue();
f32 GetAngleOfSector(vfh_state* VFH, i32 Sector);
void InitVFH(vfh_state* VFH, f32 CarRadius, i32 MaxRegionDiameter, i32 AngularSectors, f32 SafetyDistance, f32 CellUnitsToWorld);
void RunVFH(vfh_state* VFH, car* Car);
void InitVPH(vph_state* VPH, car* Car);
void RunVPH(vph_state* VPH, car* Car);
void RunDriver(driver* Driver, car* Car, vfh_state* VFH, vph_state* VPH, follow_gap* Gap);
void InitFTG(follow_gap* FTG, car* Car);
void FollowGap(follow_gap* FTG, car* Car);
#endif

