#include "path_common.h"
#include "path_car_ai.h"
#include "path_math.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>

//Dont really need to make these tunable.
#define MAX_REGION_VALUE 10
#define OBSTACLE_THRESHOLD (MAX_REGION_VALUE / 2)

f32 GetMaxVFHPrimaryValue()
{
    return MAX_REGION_VALUE * MAX_REGION_VALUE;
}

f32 GetAngleOfSector(vfh_state* VFH, i32 Sector)
{
    return (f32) (Sector / (f32) VFH->AngularSectors) * TAU32 - PI32;
}

static inline f32 Modulo(f32 Value, f32 Divisor, f32 SubRemainder)
{
    return fmodf(Divisor + SubRemainder + fmodf(Value, Divisor), Divisor) - SubRemainder;
}

static inline i32 GetIndexFromAngle(f32 Angle, i32 AngularSectors)
{
    //we want the domain to be [-PI, PI]
    f32 NewAngle = Modulo(Angle, TAU32, PI32);
    f32 SectorResolution = TAU32 / (f32) AngularSectors;
    return (i32) floorf(NewAngle / SectorResolution) + AngularSectors/2;
}

static inline i32 DetermineSector(vfh_state* VFH, f32 Angle)
{
    //we want the domain to be [-PI, PI]
    return GetIndexFromAngle(Angle, VFH->AngularSectors);
}

static inline b32 IsToTheLeftOfAngle(f32 From, f32 To)
{
    //we want the domain to be [-PI, PI]
    f32 NewFrom = Modulo(From, TAU32, PI32);
    f32 NewTo = Modulo(To, TAU32, PI32);
    f32 Difference = Modulo(NewTo - NewFrom, TAU32, PI32);
    return Difference < 0.0f;
}

static inline b32 IsToTheRightOfAngle(f32 From, f32 To)
{
    //we want the domain to be [-PI, PI]
    f32 NewFrom = Modulo(From, TAU32, PI32);
    f32 NewTo = Modulo(To, TAU32, PI32);
    f32 Difference = Modulo(NewTo - NewFrom, TAU32, PI32);
    return Difference > 0.0f;
}

static inline b32 IsBetweenAngles(f32 Angle, f32 Angle1, f32 Angle2)
{
    //we want the domain to be [-PI, PI]
    f32 NewAngle1 = Modulo(Angle1, TAU32, PI32);
    f32 NewAngle2 = Modulo(Angle2, TAU32, PI32);
    f32 NewAngle = Modulo(Angle, TAU32, PI32);
    return (NewAngle1 <= NewAngle && NewAngle <= NewAngle2) ||
           (NewAngle2 <= NewAngle && NewAngle <= NewAngle1);
}

static inline i32 Min(i32 Value1, i32 Value2)
{
    return Value1 < Value2 ? Value1 : Value2;
}

static inline i32 AbsSectorDifference(i32 Sector1, i32 Sector2, i32 AngularSectors)
{
    return Min(Min(abs(Sector1 - Sector2), abs(Sector1 - Sector2 - AngularSectors)), abs(Sector1 - Sector2 + AngularSectors));
}

static inline i32 DetermineCost(vfh_state* VFH, i32 Sector)
{
    i32 TargetCost = VFH->FollowTarget  * VFH->TargetDirectionWeight * AbsSectorDifference(Sector, VFH->TargetSector, VFH->AngularSectors);
    i32 CurrentCost = VFH->CurrentDirectionWeight * AbsSectorDifference(Sector, VFH->CurrentSector, VFH->AngularSectors);
    i32 PreviousCost = VFH->PreviousDirectionWeight * AbsSectorDifference(Sector, VFH->SelectedSector, VFH->AngularSectors);
    return TargetCost + CurrentCost + PreviousCost;
}

//note(andrew): We assume that CarRadius, and Safety Distance is already converted to cell units
void InitVFH
(vfh_state* VFH, f32 CarRadius, i32 MaxRegionDiameter, i32 AngularSectors, f32 SafetyDistance, f32 CellUnitsToWorld)
{
    Assert(MaxRegionDiameter > 0);
    VFH->RegionDiameter = MaxRegionDiameter;
    VFH->MaxRegionDiameter = MaxRegionDiameter;
    VFH->AngularSectors = AngularSectors;
    VFH->CellUnitsToWorld = CellUnitsToWorld;

    //the furthest distance between car's center and perimeter of the car
    VFH->EnlargedRadius = CarRadius + SafetyDistance;

    VFH->Threshold = 0.5f;
    VFH->MinGapSize = AngularSectors / 4;

    VFH->TargetDirectionWeight = 5;
    VFH->CurrentDirectionWeight = 2;
    VFH->PreviousDirectionWeight = 2;
    
    //TODO(andrew): We may want to use our own custom memory allocators
    VFH->Region = (i8*) malloc(MaxRegionDiameter * MaxRegionDiameter * sizeof(i8));
    VFH->RegionInfluence = (f32*) malloc(MaxRegionDiameter * MaxRegionDiameter * sizeof(f32));
    VFH->RegionDistances = (f32*) malloc(MaxRegionDiameter * MaxRegionDiameter * sizeof(f32));
    VFH->RegionDirections = (f32*) malloc(MaxRegionDiameter * MaxRegionDiameter * sizeof(f32));
    VFH->EnlargedAngles = (f32*) malloc(MaxRegionDiameter * MaxRegionDiameter * sizeof(f32));

    VFH->PrimaryHistogram = (f32*) malloc(AngularSectors * sizeof(f32));
    VFH->BinaryHistogram = (b32*) malloc(AngularSectors * sizeof(b32));
    memset(VFH->BinaryHistogram, 0, AngularSectors * sizeof(b32));
    VFH->MaskedHistogram = (b32*) malloc(AngularSectors * sizeof(b32));
    memset(VFH->MaskedHistogram, 0, AngularSectors * sizeof(b32));
    VFH->SectorGaps = (i32*) malloc(AngularSectors * sizeof(i32));
    VFH->CandidateSectors = (i32*) malloc(AngularSectors * sizeof(i32));
    VFH->CandidateCosts = (i32*) malloc(AngularSectors * sizeof(i32));

    f32 MaxDistance= (f32) (VFH->MaxRegionDiameter- 1)/2.0f;
    for(i32 CountY = 0; CountY < VFH->MaxRegionDiameter; ++CountY)
    {
       for(i32 CountX = 0; CountX < VFH->MaxRegionDiameter; ++CountX)
       {
            i32 TableIndex = CountX + CountY * VFH->MaxRegionDiameter;
            f32 RelativeCenterX = (f32) (CountX - VFH->MaxRegionDiameter/2);
            f32 RelativeCenterY = (f32) (CountY - VFH->MaxRegionDiameter/2);
            f32 Distance = sqrtf(RelativeCenterX * RelativeCenterX + RelativeCenterY * RelativeCenterY);
            Assert(Distance < 1.5f * MaxDistance);
            VFH->RegionDistances[TableIndex] = Distance;
            //Look at the VFH+ paper where they show m = c^2 * (a - bd^2) to understand what I am doing
            //this function ensures the region influence is between 0 and 1
            f32 DistanceMax = (f32) (VFH->MaxRegionDiameter - 1)/2.0f;
            if(Distance > DistanceMax)
            {
                VFH->RegionInfluence[TableIndex] = 0;
            }
            else if(Distance <= VFH->EnlargedRadius)
            {
                VFH->RegionInfluence[TableIndex] = 1;
            }
            else
            {
                f32 ParamA = MaxDistance / (MaxDistance - VFH->EnlargedRadius);
                f32 ParamB = ParamA / MaxDistance;
                VFH->RegionInfluence[TableIndex] = ParamA - ParamB * Distance;
            }
            VFH->RegionDirections[TableIndex] = atan2f(RelativeCenterY, RelativeCenterX);
            //dont try doing asinf(Radius / Distance);
            if(Distance >= VFH->EnlargedRadius)
            {
                VFH->EnlargedAngles[TableIndex] = asinf(VFH->EnlargedRadius / Distance);
            }
#if 0
            else if(Distance >= CarRadius)
            {
                VFH->EnlargedAngles[TableIndex] = PI32/2.0f;
            }
            else
            {
                VFH->EnlargedAngles[TableIndex] = PI32;
            }
#else
            else
            {
                VFH->EnlargedAngles[TableIndex] = PI32/2.0f;
            }
#endif
        }
    }

    //TODO(andrew): Figure out what the lowest upper bound capacity of the bins should be
    VFH->BinsCapacity = AngularSectors * VFH->MaxRegionDiameter * VFH->MaxRegionDiameter;
    VFH->BinsSize = 0;
    VFH->SectorBins = (i32*) malloc(VFH->BinsCapacity * sizeof(i32));
    VFH->SectorBinSizes = (i32*) malloc(VFH->AngularSectors * sizeof(i32));
    memset(VFH->SectorBins, 0, VFH->BinsCapacity * sizeof(i32));
    memset(VFH->SectorBinSizes, 0, VFH->AngularSectors * sizeof(i32));
    f32 SectorResolution = TAU32 / (f32) VFH->AngularSectors;
    //Finds the regions that overlap with each sector
    for(i32 Sector=0; Sector < VFH->AngularSectors; ++Sector)
    {
        f32 SectorAngle = GetAngleOfSector(VFH, Sector);
        for(i32 Count = 0; Count < VFH->MaxRegionDiameter * VFH->MaxRegionDiameter; ++Count)
        {
            b32 Overlaps = IsBetweenAngles(SectorAngle, 
                                           VFH->RegionDirections[Count] - VFH->EnlargedAngles[Count], 
                                           VFH->RegionDirections[Count] + VFH->EnlargedAngles[Count]);

            if(Overlaps)
            {
                VFH->SectorBinSizes[Sector]++;
                Assert(VFH->BinsSize < VFH->BinsCapacity);
                VFH->SectorBins[VFH->BinsSize++] = Count;
            }
        }
    }
}

//Implementation of the algorithm: Enhanced Vector Field Histogram (VFH+)
//note(To reader): VFH+ paper has a couple of notational mistakes you should be wary of.
void RunVFH (vfh_state* VFH, car* Car)
{
    Assert(VFH->RegionDiameter <= VFH->MaxRegionDiameter)
    //note(andrew): There is probably a better way to update region
    memset(VFH->Region, 0, VFH->RegionDiameter*VFH->RegionDiameter*sizeof(i8));
    for(i32 Count=0; Count < Car->ScanSize; ++Count)
    {
        f32 Distance = Car->ScanDistances[Count] * VFH->CellUnitsToWorld + VFH->EnlargedRadius;

        if(Distance > VFH->RegionDiameter/2 - 1)
        {
            continue;
        }

        f32 SampleAngle = GetAngleOfScanSample(Count, Car->ScanSize, Car->MinSensorAngle, Car->MaxSensorAngle);
        i32 MappedSampleX = (i32) (Distance * cosf(SampleAngle)) + VFH->RegionDiameter/2;
        i32 MappedSampleY = (i32) (Distance * sinf(SampleAngle)) + VFH->RegionDiameter/2;
        Assert(MappedSampleX > 0 && MappedSampleY > 0);
        Assert(MappedSampleX < VFH->RegionDiameter && MappedSampleY < VFH->RegionDiameter);
        VFH->Region[MappedSampleX + MappedSampleY * VFH->RegionDiameter] = MAX_REGION_VALUE;
    }


    f32 SectorResolution = TAU32 / (f32) VFH->AngularSectors;

    f32 SteerRadius = ApproximateEffectiveTurnRadius(Car->Velocity, Car->Height, Car->MaxTurnAngle, Car->WheelFriction);
    f32 LeftSteerRadius = SteerRadius;
    f32 RightSteerRadius = SteerRadius;
    f32 LeftSteerCenterX = 0;
    f32 LeftSteerCenterY = LeftSteerRadius;
    f32 RightSteerCenterX = 0;
    f32 RightSteerCenterY = -RightSteerRadius;

    f32 SmallEpsilon = (1.0f / 1024.0f);
#if 0
    f32 LeftAngleLimit = PI32 - SmallEpsilon;
    f32 RightAngleLimit = -(PI32 - SmallEpsilon);
#else
    f32 LeftAngleLimit = Car->MaxSensorAngle - SmallEpsilon;
    f32 RightAngleLimit = Car->MinSensorAngle + SmallEpsilon;
#endif

    for(i32 Count = 0; Count < VFH->RegionDiameter * VFH->RegionDiameter; ++Count)
    {

        if(VFH->Region[Count] <= OBSTACLE_THRESHOLD)
        {
            continue;
        }

        i32 CountX = Count % VFH->RegionDiameter;
        i32 CountY = Count / VFH->RegionDiameter;
        f32 CellX = (f32) CountX - VFH->RegionDiameter/2;
        f32 CellY = (f32) CountY - VFH->RegionDiameter/2;

        f32 RightDistanceSquared = (RightSteerCenterX - CellX) * (RightSteerCenterX - CellX) + 
                           (RightSteerCenterY - CellY) * (RightSteerCenterY - CellY); 
        b32 RightLegal = IsToTheRightOfAngle(VFH->RegionDirections[Count], 0);
        b32 LeftLegal = IsToTheLeftOfAngle(VFH->RegionDirections[Count], RightAngleLimit);
        f32 RightMinDist = (RightSteerRadius + VFH->EnlargedRadius) * (RightSteerRadius + VFH->EnlargedRadius);
        if(RightLegal && LeftLegal && RightDistanceSquared < RightMinDist)
        {
            RightAngleLimit = VFH->RegionDirections[Count];
        }

        f32 LeftDistanceSquared = (LeftSteerCenterX - CellX) * (LeftSteerCenterX - CellX) + 
                           (LeftSteerCenterY - CellY) * (LeftSteerCenterY - CellY); 
        LeftLegal = IsToTheLeftOfAngle(VFH->RegionDirections[Count], 0);
        RightLegal = IsToTheRightOfAngle(VFH->RegionDirections[Count], LeftAngleLimit);  
        f32 LeftMinDist = (LeftSteerRadius + VFH->EnlargedRadius) * (LeftSteerRadius + VFH->EnlargedRadius);
        if(LeftLegal && RightLegal && LeftDistanceSquared < LeftMinDist)
        {
            LeftAngleLimit = VFH->RegionDirections[Count];
        }
    }

    Assert(RightAngleLimit <= 0 && RightAngleLimit > -PI32);
    Assert(LeftAngleLimit >= 0 && LeftAngleLimit < PI32);


    VFH->GapExists = false;
    f32 MaxPrimaryValue = GetMaxVFHPrimaryValue();

    //Primary Polar Histogram (First Stage of VFH+)
    memset(VFH->PrimaryHistogram, 0, VFH->AngularSectors * sizeof(f32));
    i32 StartCount = 0;
    f32 AverageBinSize = (f32) VFH->BinsSize / (f32) VFH->AngularSectors;
    for(i32 Sector=0; Sector < VFH->AngularSectors; ++Sector)
    {
        i32 EndCount = StartCount + VFH->SectorBinSizes[Sector];
        f32 SectorWeight = (AverageBinSize / (f32) VFH->SectorBinSizes[Sector]);
        for(i32 Count = StartCount; Count < EndCount; ++Count)
        {
            Assert(Count < VFH->BinsSize);
            i32 RegionIndex = VFH->SectorBins[Count];
            i32 RegionSquared = VFH->Region[RegionIndex] * VFH->Region[RegionIndex];
            VFH->PrimaryHistogram[Sector] = fmaxf((f32) (RegionSquared * VFH->RegionInfluence[RegionIndex]), 
                                                  VFH->PrimaryHistogram[Sector]);
        }
        //VFH->PrimaryHistogram[Sector] *= SectorWeight;
        StartCount = EndCount;
    }

    for(i32 Sector=0; Sector < VFH->AngularSectors; ++Sector)
    {
        f32 SectorAngle = GetAngleOfSector(VFH, Sector);
        //Binary Polar Histogram (Second Stage of VFH+)
        b32 AboveThreshold= VFH->PrimaryHistogram[Sector] > VFH->Threshold * MaxPrimaryValue;
        VFH->BinaryHistogram[Sector] = AboveThreshold;

        //Masked Polar Histogram (Third Stage of VFH+)
        b32 IsWithinLimits = IsBetweenAngles(SectorAngle, RightAngleLimit, LeftAngleLimit);
        VFH->MaskedHistogram[Sector] = VFH->BinaryHistogram[Sector] || !IsWithinLimits;
        VFH->GapExists = VFH->GapExists || !VFH->MaskedHistogram[Sector];
    }

    if(!VFH->GapExists)
    {
        return;
    }

    //Choosing the Steering Direction (Fourth and Final Stage of VFH+)
    b32 InGap = false;
    i32 StartOfGap;
    i32 GapSize = 0;
    memset(VFH->SectorGaps, 0, VFH->AngularSectors * sizeof(f32));
    for(i32 Sector=0; Sector < VFH->AngularSectors; ++Sector)
    {
        if(!VFH->MaskedHistogram[Sector])
        {
            if(!InGap)
            {
                InGap = true;
                StartOfGap = Sector;
            }
            ++GapSize;
            if(Sector == VFH->AngularSectors-1)
            {
                for(i32 GapSector = StartOfGap; GapSector < Sector; ++GapSector)
                {
                    VFH->SectorGaps[GapSector] = GapSize;
                }
            }
        }
        else
        {
            if(InGap)
            {
                for(i32 GapSector=StartOfGap; GapSector < Sector; ++GapSector)
                {
                    VFH->SectorGaps[GapSector] = GapSize;
                }
                InGap = false;
            }
            VFH->SectorGaps[Sector] = 0;
            GapSize = 0;
        }
    }

    VFH->NumberOfCandidates = 0;
    b32 PreviousAdded = false;
    b32 CurrentAdded = false;
    b32 TargetAdded = false;
    //note(andrew): CurrentSector is always in the middle because sectors are relative to the car. 
    VFH->CurrentSector = VFH->AngularSectors / 2;
    VFH->TargetSector = DetermineSector(VFH, VFH->TargetHeading);
    i32 MinCost = (1 << 31) - 1;
    i32 MinSector = 0;
    for(i32 Sector=0; Sector < VFH->AngularSectors;)
    {
        i32 GapSize = VFH->SectorGaps[Sector];
        if(!GapSize)
        {
            ++Sector;
            continue;
        }
        if(GapSize < VFH->MinGapSize)
        {
            i32 CenterSector = Sector + GapSize/2;
            VFH->CandidateSectors[VFH->NumberOfCandidates] = CenterSector;
            VFH->CandidateCosts[VFH->NumberOfCandidates] = DetermineCost(VFH, CenterSector);
            ++VFH->NumberOfCandidates;
        }
        else
        {
            i32 RightSector = Sector + VFH->MinGapSize/2;
            f32 RightSectorAngle = GetAngleOfSector(VFH, RightSector);
            VFH->CandidateSectors[VFH->NumberOfCandidates] = RightSector;
            VFH->CandidateCosts[VFH->NumberOfCandidates] = DetermineCost(VFH, RightSector);
            ++VFH->NumberOfCandidates;

            i32 LeftSector = Sector + GapSize - VFH->MinGapSize/2;
            Assert(LeftSector > 0);
            f32 LeftSectorAngle = LeftSector * SectorResolution;
            VFH->CandidateSectors[VFH->NumberOfCandidates] = LeftSector;
            VFH->CandidateCosts[VFH->NumberOfCandidates] = DetermineCost(VFH, LeftSector);
            ++VFH->NumberOfCandidates;

            if(!VFH->FollowTarget)
            {
                if(VFH->CurrentSector >= Sector && VFH->CurrentSector <= Sector + GapSize)
                {
                    //It isn't possible for an angle to lie between more than one gap
                    //If this assertions fails, we have a problem
                    Assert(!CurrentAdded);
                    CurrentAdded = true;
                    VFH->CandidateSectors[VFH->NumberOfCandidates] = VFH->CurrentSector;
                    VFH->CandidateCosts[VFH->NumberOfCandidates] = DetermineCost(VFH, VFH->CurrentSector);
                    ++VFH->NumberOfCandidates;
                }
                else if(VFH->SelectedSector >= Sector && VFH->SelectedSector <= Sector + GapSize)
                {
                    Assert(!PreviousAdded);
                    PreviousAdded = true;
                    VFH->CandidateSectors[VFH->NumberOfCandidates] = VFH->SelectedSector;
                    VFH->CandidateCosts[VFH->NumberOfCandidates] = DetermineCost(VFH, VFH->SelectedSector);
                    ++VFH->NumberOfCandidates;
                }
            }
            else if(VFH->TargetSector >= Sector && VFH->TargetSector <= Sector + GapSize)
            {
                Assert(!TargetAdded);
                VFH->CandidateSectors[VFH->NumberOfCandidates] = VFH->TargetSector;
                VFH->CandidateCosts[VFH->NumberOfCandidates] = DetermineCost(VFH, VFH->TargetSector);
                ++VFH->NumberOfCandidates;
            }
        }
        Sector += GapSize;
    }

    for(i32 Count=0; Count < VFH->NumberOfCandidates; ++Count)
    {
        if(VFH->CandidateCosts[Count] < MinCost)
        {
            MinCost = VFH->CandidateCosts[Count];
            MinSector = VFH->CandidateSectors[Count];
        }
    }

    VFH->SelectedSector = MinSector;

    if(VFH->SelectedSector == VFH->TargetSector)
    {
        VFH->SelectedAngle = VFH->TargetHeading;
    }
    else if(VFH->SelectedSector == VFH->CurrentSector)
    {
        VFH->SelectedAngle = GetAngleOfSector(VFH, VFH->CurrentSector);
    }
    else
    {
        VFH->SelectedAngle = GetAngleOfSector(VFH, VFH->SelectedSector);
    }
}

void InitVPH(vph_state* VPH, car* Car)
{
    VPH->ModifiedDistances = (f32*) malloc(Car->ScanSize * sizeof(f32));
    VPH->GroupStartIndices = (i32*) malloc(Car->ScanSize * sizeof(i32));
    VPH->GroupID = (i32*) malloc(Car->ScanSize * sizeof(i32));
    VPH->IsConcave = (b32*) malloc(Car->ScanSize * sizeof(b32));
    VPH->Thresholds = (b32*) malloc(Car->ScanSize * sizeof(b32));
}

//WIP Implementation of Enhanced Vector Polar Histogram (VPH+) from authors Jianwei Gong et al.
void RunVPH(vph_state* VPH, car* Car)
{
    f32 CarRadius = GetCarRadius(Car);
    f32 AnglePrecision= GetScanAnglePrecision(Car->MinSensorAngle, Car->MaxSensorAngle, Car->ScanSize);
    //A. Modification of Original Information
    for (i32 Count1 = 0; Count1 < Car->ScanSize; ++Count1) 
    {
        VPH->ModifiedDistances[Count1] = Car->ScanDistances[Count1];
        f32 ScanAngle1 = Count1 * AnglePrecision;
        for (i32 Count2 = 0; Count2 < Car->ScanSize; ++Count2)
        {
            f32 ScanAngle2 = Count2 * AnglePrecision;
            f32 AngleDiff = fminf(fabs(ScanAngle1 - ScanAngle2), TAU32 - fabs(ScanAngle1 - ScanAngle2));
            f32 SinAngleDist = Car->ScanDistances[Count2] * sinf(AngleDiff);
            f32 CosAngleDist = Car->ScanDistances[Count2] * cosf(AngleDiff);
            f32 Distance = CosAngleDist;
            f32 SmallEpsilon = 1.0f / 1024.0f;
            if(AngleDiff > PI32/2-SmallEpsilon || SinAngleDist > CarRadius || CosAngleDist >= Car->ScanDistances[Count1])
            {
                Distance = Car->ScanDistances[Count1];
            }
            VPH->ModifiedDistances[Count1] = fminf(VPH->ModifiedDistances[Count1], Distance);
        }
        VPH->ModifiedDistances[Count1] = fmaxf(0.0f, VPH->ModifiedDistances[Count1] - CarRadius);
    }

    //B. Grouping Obstacles into Blocks
    Assert(VPH->DistanceThreshold > CarRadius)
    VPH->NumberOfGroups = 1;
    VPH->GroupStartIndices[0] = 0;
    for (i32 Count=0; Count < Car->ScanSize-1;++Count)
    {
        VPH->GroupID[Count] = VPH->NumberOfGroups-1;
        f32 Distance1 = Car->ScanDistances[Count];
        i32 Count2 = Count+1;
        f32 Distance2 = Car->ScanDistances[Count2];
        f32 ConjoinDistance = sqrtf(Distance1*Distance1 + Distance2*Distance2 - 2*Distance1*Distance2*cosf(AnglePrecision));
        if(ConjoinDistance >= VPH->DistanceThreshold)
        {
            VPH->GroupStartIndices[VPH->NumberOfGroups++] = Count+1;
        }
    }
    VPH->GroupID[Car->ScanSize-1] = VPH->NumberOfGroups-1;

    //C. Construction of Symbol Function 
    memset(VPH->IsConcave, 0, sizeof(i32) * Car->ScanSize);
    for(i32 GroupCount=1; GroupCount < VPH->NumberOfGroups-1; ++GroupCount)
    {
        b32 LessThanPrevGroup = Car->ScanDistances[VPH->GroupStartIndices[GroupCount]] < 
                                Car->ScanDistances[VPH->GroupStartIndices[GroupCount]-1];

        b32 LessThanNextGroup = Car->ScanDistances[VPH->GroupStartIndices[GroupCount+1]-1] < 
                                Car->ScanDistances[VPH->GroupStartIndices[GroupCount+1]];
        VPH->IsConcave[GroupCount] = LessThanNextGroup && LessThanPrevGroup;
    }

    //D. Construction of Threshold Function
    f32 TurnRadius = ApproximateEffectiveTurnRadius(Car->MotorSpeed, Car->Height, Car->MaxTurnAngle, Car->WheelFriction);
    f32 SafeDistance = VPH->SafetyCoeff * (VPH->SpeedOutput / VPH->MaxSpeed * TurnRadius);
    for(i32 Count=0; Count < Car->ScanSize; ++Count)
    {
        VPH->Thresholds[Count] = VPH->ModifiedDistances[Count] >= SafeDistance;
    }

    //E. Construction of Cost Function 
    f32 BestScore = 0;
    f32 BestAngle = 0;
    f32 BestDistance = 0;
    for(i32 Count = 0; Count < Car->ScanSize; ++Count)
    {
        f32 Angle = GetAngleOfScanSample(Count, Car->ScanSize, Car->MinSensorAngle, Car->MaxSensorAngle);
        f32 TurnAngleDiff = fminf(fabs(Angle - Car->TurnAngle), TAU32 - fabs(Angle - Car->TurnAngle));
        f32 Distance = VPH->ModifiedDistances[Count];
        f32 NumeratorScore = (!VPH->IsConcave[VPH->GroupID[Count]]) * VPH->Thresholds[Count] * Distance;
        f32 DenominatorScore = 1.0f + TurnAngleDiff;
        f32 Score = NumeratorScore / DenominatorScore;
        if(Score > BestScore)
        {
            BestAngle = Angle;
            BestScore = Score;
            BestDistance = Distance;
        }
        //printf("Count: %03d Raw Distance: %f Mod Distance: %f Group ID: %d Convex: %d Threshold: %d\n", Count, Car->ScanDistances[Count], VPH->ModifiedDistances[Count], VPH->GroupID[Count], !VPH->IsConcave[VPH->GroupID[Count]], VPH->Thresholds[Count]);
    }
    VPH->TurnAngleOutput = BestAngle;
    VPH->SpeedOutput = fminf(VPH->MaxSpeed, sqrtf(2 * VPH->Accel * BestDistance));
}

void InitFTG(follow_gap* FTG, car* Car)
{
    FTG->lidar_gap = (i32*) malloc(Car->ScanSize * sizeof(i32));
    FTG->lidar_gap_size = Car->ScanSize;
    FTG->Obstacles = (f32*) malloc(Car->ScanSize * sizeof(f32));
    FTG->NumberOfObstacles = 0;
}

static f32 MinArrayVal(f32* arry, i32 start, i32 end)
{
    f32 min = 99999;
    for (i32 i = start; i < end; i++)
    {
        min = fminf(min, arry[i]);
    }
    return min;
}

void FindMaxGap(follow_gap* FTG, car* Car) 
{
    FTG->NumberOfObstacles = 0;
    for (int i = 0; i < Car->ScanSize - 1; i++)
    {
        if (Car->ScanDistances[i] < FTG->scan_dist)
        {
            if (fabs(Car->ScanDistances[i + 1] - Car->ScanDistances[i]) > FTG->Threshold)
            {
                FTG->lidar_gap[i] = 0;
            }
            else
            {
                FTG->lidar_gap[i] = 1;
            }
        }
        else 
        {
            FTG->lidar_gap[i] = 0;
        }
    }
    FTG->lidar_gap[FTG->lidar_gap_size - 1] = 1;

    std::vector<gapStruct> gapMap;

    i32 startGap = 0, endGap = 0, startObs = 0, endObs = 0, gapWidth = 0;
    for (int i = 1; i < FTG->lidar_gap_size; i++)
    {
        if (FTG->lidar_gap[i - 1] == 1 && FTG->lidar_gap[i] == 0)
        {
            startGap = i;
            if (FTG->lidar_gap[i] == 0 && i == FTG->lidar_gap_size - 1)
            {
                endGap = startGap;
                gapWidth = endGap - startGap + 1;
                gapStruct gap;
                gap.gapStart = startGap;
                gap.gapEnd = endGap;
                gap.gapWidth = gapWidth;
                gapMap.push_back(gap);
            }
        }
        else if (FTG->lidar_gap[i - 1] == 0 && FTG->lidar_gap[i] == 1)
        {
            endGap = i - 1;
            gapWidth = endGap - startGap + 1;
            gapStruct gap;
            gap.gapStart = startGap;
            gap.gapEnd = endGap;
            gap.gapWidth = gapWidth;
            gapMap.push_back(gap);
        }
        else if (FTG->lidar_gap[i] == 0 && i == FTG->lidar_gap_size - 1)
        {
            endGap = i;
            gapWidth = endGap - startGap + 1;
            gapStruct gap;
            gap.gapStart = startGap;
            gap.gapEnd = endGap;
            gap.gapWidth = gapWidth;
            gapMap.push_back(gap);
        }
    }
    for (int i = 1; i < FTG->lidar_gap_size; i++)
    {
        if (FTG->lidar_gap[i - 1] == 0 && FTG->lidar_gap[i] == 1)
        {
            startObs = i;
            if (FTG->lidar_gap[i] == 1 && i == FTG->lidar_gap_size - 1)
            {
                FTG->Obstacles[FTG->NumberOfObstacles++] = Car->ScanDistances[i - 1];
            }
        }
        else if (FTG->lidar_gap[i - 1] == 1 && FTG->lidar_gap[i] == 0)
        {
            endObs = i - 1;
            FTG->Obstacles[FTG->NumberOfObstacles++] = Car->ScanDistances[(startObs+endObs)/2];
        }
        else if (FTG->lidar_gap[i] == 1 && i == FTG->lidar_gap_size - 1)
        {
            FTG->Obstacles[FTG->NumberOfObstacles++] = Car->ScanDistances[(i + startObs) / 2];
        }
    }
    FTG->max_gapWidth = 0;
    for (int i = 0; i < gapMap.size(); i++)
    {
        if (gapMap[i].gapWidth >= FTG->max_gapWidth)
        {
            FTG->max_gapWidth = gapMap[i].gapWidth;
            FTG->max_gap_start = gapMap[i].gapStart;
            FTG->max_gap_end = gapMap[i].gapEnd;
        }
    }
}

f32 FindHeading(follow_gap* FTG, car* Car)
{
    f32 goal_angle = FTG->max_gapCenter;
    f32 Alpha = 10;
    f32 Beta = 1;
    if (FTG->max_gapWidth > FTG->MinGapSize)
    {
        if (!FTG->NumberOfObstacles)
        {
            return 0;
        }
        f32 numerator = Alpha / MinArrayVal(FTG->Obstacles, 0, FTG->NumberOfObstacles) * FTG->max_gapCenter + Beta * goal_angle;
        f32 denominator = Alpha / MinArrayVal(FTG->Obstacles, 0, FTG->NumberOfObstacles) + Beta;
        f32 theta_final = numerator / denominator;      // - math.pi/2 if considering 0 as y-axis
        return theta_final;
    }
    else
    {
        return 0;
    }
}

#if 0
    def _getFrontDist(self, angle=0):
        middle = len(self.car.lidar)//2
        # relative angle ranges from -pi/2 to pi/2
        lidar_beam_angle = (math.pi / len(self.car.lidar))
        #determine the range of values to scan
        scope = int(math.atan(1) / lidar_beam_angle) + 1
        # this converts relative angle to corresponding LIDAR index
        index = int(angle / lidar_beam_angle + middle)
        
        front_dist = self.car.lidar[index]
        for i in range(-scope,scope+1):
            if(self.car.lidar[index+i]*math.sin(abs(i)*lidar_beam_angle) < self.car.carLength/2):
                if(self.car.lidar[index+i] < front_dist):
                    front_dist = self.car.lidar[index+i]
        return front_dist
#endif

f32 GetFrontDist(car* Car, f32 angle)
{
    i32 MiddleIndex = floorf(Car->ScanSize/2);
    f32 LidarBeamAngle = GetScanAnglePrecision(Car->MinSensorAngle, Car->MaxSensorAngle, Car->ScanSize);
    i32 Scope = (i32) (atanf(0.2f) / LidarBeamAngle) + 1;
    i32 Index = (i32) (angle / LidarBeamAngle) +  MiddleIndex;

    f32 FrontDist = Car->ScanDistances[Index];

    for(i32 Count = -Scope; Count < Scope+1; ++Count)
    {
        if(Car->ScanDistances[Index+Count] * sinf(abs(Count)*LidarBeamAngle) < Car->Height * 0.5f &&
           Car->ScanDistances[Index+Count] < FrontDist)
        {
            FrontDist = fminf(FrontDist, Car->ScanDistances[Index+Count]);
        }
    }
    return FrontDist;
}

void FollowGap(follow_gap* FTG, car* Car)
{
    FindMaxGap(FTG,Car);

    i32 center_angle = Car->ScanSize / 2;
    FTG->center_gap_index = (FTG->max_gap_end + FTG->max_gap_start) / 2;
    FTG->max_gapCenter = GetAngleOfScanSample(FTG->center_gap_index, Car->ScanSize, Car->MinSensorAngle, Car->MaxSensorAngle);

    f32 HeadingAngle = FindHeading(FTG, Car);

    f32 right = MinArrayVal(Car->ScanDistances, 0, 90);
    f32 left = MinArrayVal(Car->ScanDistances, Car->ScanSize - 90, Car->ScanSize);
    if (left < FTG->SafetyDistance && left < right)
    {
        FTG->TurnAngleOutput = HeadingAngle - PI32 / 8;
    }
    else if (right < FTG->SafetyDistance && right < left)
    {
        FTG->TurnAngleOutput = HeadingAngle + PI32 / 8;
    }
    else
    {
        FTG->TurnAngleOutput = HeadingAngle;
    }

    f32 diff = fabs(FTG->TurnAngleOutput - Car->TurnAngle);
    if(FTG->TurnAngleOutput > Car->TurnAngle)
    {
        ChangeTurnAngle(Car, Car->TurnAngle + diff);
    }
    else if(FTG->TurnAngleOutput < Car->TurnAngle)
    {
        ChangeTurnAngle(Car, Car->TurnAngle - diff);
    }

    f32 front_dist = GetFrontDist(Car, Car->TurnAngle);

    f32 velocity = (front_dist - 2.5f) / 7.5f * (FTG->speed_factor - 0.2f) + 0.2f;
    if (velocity < FTG->MinSpeed)//min motor speed
    {
        velocity = FTG->MinSpeed;
    }
    else if (velocity > FTG->MaxSpeed)//max motor speed
    {
        velocity = FTG->MaxSpeed;
    }
#if 0
    if (front_dist < FTG->StopThreshold)//stop the breaks
    {
        velocity = 0;
    }
#endif
    FTG->SpeedOutput = velocity;
}

void RunDriver(driver* Driver, car* Car, vfh_state* VFH, vph_state* VPH, follow_gap* FTG)
{
    switch(Driver->AvoidanceMethod)
    {
        case Avoid_VFH:
        {
            Driver->SelectedOption = 0;
            f32 MinSpeed = Driver->SpeedOptions[Driver->NumberOfOptions-1];
            f32 MinMaxSpeedDiff = Driver->SpeedOptions[0] - MinSpeed;
            f32 ThresholdSlope = (Driver->MinVFHThreshold - Driver->MaxVFHThreshold) / MinMaxSpeedDiff ;
            for(i32 Option=0; Option < Driver->NumberOfOptions; ++Option)
            {
                VFH->Threshold = ThresholdSlope * (Driver->SpeedOptions[Option] - MinSpeed) + Driver->MaxVFHThreshold;
                RunVFH(VFH, Car);
                if(VFH->GapExists)
                {
                    Driver->SelectedOption = Option;
                    break;
                }
            }
            f32 RawMotorSpeed = Driver->SpeedOptions[Driver->SelectedOption];
            Driver->StopCar = !VFH->GapExists;
            ChangeTurnAngle(Car, VFH->SelectedAngle);
            Car->MotorSpeed = fmaxf(RawMotorSpeed * cosf(VFH->SelectedAngle), MinSpeed);
            if(Driver->StopCar)
            {
                Car->MotorSpeed = 0.0f;
            }
        } break;
        case Avoid_VPH:
        {
            RunVPH(VPH, Car);
            Car->MotorSpeed = VPH->SpeedOutput;
            ChangeTurnAngle(Car, VPH->TurnAngleOutput);
        } break;
        case Avoid_FTG:
        {
            FollowGap(FTG, Car);
            Car->MotorSpeed = FTG->SpeedOutput;
            ChangeTurnAngle(Car, FTG->TurnAngleOutput);
        } break;
        default: InvalidCodePath;
    }
}
