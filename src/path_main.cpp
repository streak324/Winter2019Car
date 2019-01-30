#include <stdio.h>
#include <iostream>
#include <fstream>
#include <SDL.h>
#undef main
#include <string>
#include "glad.h"
#include "path_common.h"
#include "path_platform.h"
#include "path_heaps.h"
#include "path_navigator.h"
#include "path_math.h"
#include "path_render.h"
#include "path_car.h"
#include "path_car_ai.h"

#define SecondsPerFrame 1.0f / 60.0f

static f32 TargetX;
static f32 TargetY;
static v3 CameraVelocity = {};
static v3 CameraPosition = {};
static const f32 CamSpeed = 20;
static std::vector<u32> Path;
static navigator Navigator;
static grid Grid;
static u64 PlatformCycleCount;
static car Car = {};
static const f32 PixelsPerMapUnit = 32.0f;
static const f32 VFHUnitsPerMapUnit = 2.0f;
static const f32 PixelsPerVFHUnit = PixelsPerMapUnit / VFHUnitsPerMapUnit;

enum DebugScreenState
{
    DebugState_Paths,
    DebugState_SubgoalConnections
};

const char SimpleVS[] =
{
    "#version 330 core\n"
    "layout (location = 0) in vec2 aPos;\n"
    "layout (location = 1) in vec4 aColor;\n"
    "uniform mat4 ProjView;\n"
    "out vec4 OutVertexColor;\n"
    "void main()\n"
    "{\n"
        "OutVertexColor = aColor;\n"
        "gl_Position = ProjView * vec4(aPos.x, aPos.y, 0.0f, 1.0);\n"
    "}\n"
};

const char SimpleFS[] =
{
    "#version 330 core\n"
    "in vec4 OutVertexColor;\n"
    "out vec4 FragColor;\n"
    "void main()\n"
    "{\n"
    "    FragColor = OutVertexColor;\n"
    "}\n" 
};

const char TextureVS[] =
{
    "#version 450 core\n"
    "layout (location = 0) in vec2 aPos;\n"
    "layout (location = 1) in vec4 aColor;\n"
    "layout (location = 2) in vec2 aTex;\n"
    "uniform mat4 ProjView;\n"

    "out vec4 OutVertexColor;\n"
    "out vec2 InTexCoords;\n"
    "void main()\n"
    "{\n"
        "OutVertexColor = aColor;\n"
        "InTexCoords = aTex;\n"
        "gl_Position = ProjView * vec4(aPos.x, aPos.y, 0.0f, 1.0);\n"
    "}\n"
};

const char TextureFS[] =
{
    "#version 450 core\n"
    "in vec4 OutVertexColor;\n"
    "in vec2 InTexCoords;\n"
    "out vec4 FragColor;\n"
    "uniform sampler2D MyTexture;\n"
    "void main()\n"
    "{\n"
    "    FragColor = OutVertexColor * texture(MyTexture, InTexCoords);\n"
    "}\n" 
};

static DebugScreenState CurrentDebugState = DebugState_Paths;

static grid CreateGridFromGPPCBenchmark(const char* Filename)
{
    grid Grid = {};
    std::ifstream File;
    File.open(Filename);
    std::string Line;

    std::getline(File, Line);

    std::getline(File, Line);
    sscanf_s(Line.c_str(), "height %u", &Grid.Height); 
    std::getline(File, Line);
    sscanf_s(Line.c_str(), "width %u", &Grid.Width); 
    std::getline(File, Line);

    Grid.Data = new u8[Grid.Height*Grid.Width];
    memset(Grid.Data, 0, Grid.Height*Grid.Width*sizeof(u8));

    u32 Index = 0;
    while(std::getline(File, Line))
    {
        for(u32 StringIndex=0; StringIndex < Line.size(); ++StringIndex)
        {
            char Type = Line[StringIndex];
            switch(Type)    
            {
                case '@':
                case 'T':
                case 'W':
                    u32 GridX = Index % Grid.Width;
                    u32 GridY = Index / Grid.Width;
                    Grid.Data[GridX + (Grid.Height - GridY - 1) * Grid.Width] = BLOCKED_CELL;
                break;
            }
            ++Index;
        }
    }
    return Grid;
}

static grid CreateGridFromPGM(const char* Filename)
{
    std::ifstream File;
    File.open(Filename, std::ios::binary);
    std::string Line;
    grid Grid = {};

    std::getline(File, Line);
    std::getline(File, Line);
    std::getline(File, Line);
    sscanf_s(Line.c_str(), "%u %u", &Grid.Width, &Grid.Height); 
    std::getline(File, Line);

    Grid.Data = new u8[Grid.Height*Grid.Width];
    memset(Grid.Data, 0, Grid.Height*Grid.Width*sizeof(u8));

    u32 Index = 0;
    while(File.good())
    {
        u8 Type = File.get();
        if(Type <= 0xCD)
        {
            Grid.Data[Index] = BLOCKED_CELL;
        }
        ++Index;
    }
    return Grid;
}

u64 GetHardwareTime()
{
    return SDL_GetPerformanceCounter();
}

f32 GetSecondsElapsed(u64 PrevCounter, u64 CurrentCounter)
{
    return ((f32) (CurrentCounter - PrevCounter)) / PlatformCycleCount;
}

static void DrawBinaryHistogram(f32 PosX, f32 PosY, b32* Histogram, i32 Size, v4 BGColor)
{
    ColorPolygon(BGColor.X, BGColor.Y, BGColor.Z, BGColor.W);
    f32 BorderWidth = 2.0f;
    f32 CellWidth = 6.0f;
    f32 CellHeight = 128.0f;
    f32 Padding = 16.0f;
    f32 HistogramWidth = Size * (BorderWidth + CellWidth) + 2 * Padding;
    f32 HistogramHeight = CellHeight + 2 * Padding;

    TranslatePolygon(PosX, PosY);
    PushQuad(HistogramWidth, HistogramHeight, false);

    for(i32 Count=0; Count < Size; ++Count)
    {
        ColorPolygon(0.0f, 0.0f, 0.0f, 1.0f);
        TranslatePolygon(PosX + Padding + Count * (CellWidth + BorderWidth) - BorderWidth/2, PosY + Padding);
        if(Histogram[Count])
        {
            PushQuad(CellWidth+BorderWidth, CellHeight, false);
        }
        else
        {
            PushQuad(CellWidth+BorderWidth, 8.0f, false);
        }

        ColorPolygon(1.0f, 1.0f, 1.0f, 1.0f);
        TranslatePolygon(PosX + Padding + Count * (CellWidth + BorderWidth), PosY + Padding);
        if(Histogram[Count])
        {
            PushQuad(CellWidth, CellHeight, false);
        }
        else
        {
            PushQuad(CellWidth, 8.0f, false);
        }
    }
}

int main()
{
    if(SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }
#if defined (_WIN32) || defined (__WIN32) || defined(WIN32) || defined(_MSC_VER)
    i32 ScreenWidth = (i32) GetSystemMetrics(SM_CXSCREEN);
    i32 ScreenHeight = (i32) GetSystemMetrics(SM_CYSCREEN);
#else
    SDL_DisplayMode Display;
    SDL_GetDesktopDisplayMode(0, &Display);
    i32 ScreenWidth = Display.w;
    i32 ScreenHeight = Display.h;
#endif

    SDL_GL_SetAttribute(SDL_GL_ACCELERATED_VISUAL, 1);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    SDL_Window* Window = SDL_CreateWindow("Pathfinding Demo", SDL_WINDOWPOS_UNDEFINED,SDL_WINDOWPOS_UNDEFINED, ScreenWidth, ScreenHeight, SDL_WINDOW_BORDERLESS | SDL_WINDOW_OPENGL);
    if (!Window)
    {
        std::cout << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }
    SDL_GLContext GLContext = SDL_GL_CreateContext(Window);
    SDL_GL_SetSwapInterval(1);

    if(!gladLoadGLLoader(SDL_GL_GetProcAddress))
    {
        printf("Couldn't Load GL functions");
        SDL_DestroyWindow(Window);
        SDL_Quit();
    }
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    PlatformCycleCount = SDL_GetPerformanceFrequency();
    static const i32 LogicalCPUCount = SDL_GetCPUCount();

    Grid = CreateGridFromGPPCBenchmark("assets/arena.map");

    Navigator = {};
    u64 NavigateSetupStart = SDL_GetPerformanceCounter();
    SetupNavigator(&Navigator, Grid);
    f32 SetupTime = (f32) (SDL_GetPerformanceCounter() - NavigateSetupStart) / PlatformCycleCount;
    printf("%f seconds to finish navigator setup\n", SetupTime);

    Navigator.ExpandMethod = NavExpand_Immediate;
    Navigator.OrderMethod = NavOrder_AStar;
    Navigator.JumpBound = 1024;

    Car = {};
    Car.CenterX = 3.0f;
    Car.CenterY = 3.0f;
    Car.Width = 1.0f;
    Car.Height = 2.0f;
    Car.WheelFriction = 10;
    Car.Mass = 1.0;
    Car.Angle = 0;
    Car.MaxSensorRange = 20;
    Car.MinSensorAngle = -PI32/2;
    Car.MaxSensorAngle = PI32/2;
    Car.MaxTurnAngle = PI32/4;
    Car.ScanSize = 180;
    Car.ScanDistances = (f32*) malloc(Car.ScanSize * sizeof(f32));

    driver Driver = {};
#if 1
    Driver.MinVFHThreshold = 0.3f;
    Driver.MaxVFHThreshold = 0.8f;
    f32 DriverSpeedOptions[] = {12.0, 8.0f, 4.0f};
    Driver.SpeedOptions = DriverSpeedOptions;
    Driver.NumberOfOptions = ArrayCount(DriverSpeedOptions);
#endif
    Driver.AvoidanceMethod = Avoid_VPH;
    b32 UseDriver = false;

    f32 CarRadius = sqrtf(Car.Width*Car.Width/4 + Car.Height*Car.Height/4);

    vfh_state VFH = {};
    u64 VFHStartupCounter = GetHardwareTime();
    f32 SafetyDistance = 0.0f;
    InitVFH(&VFH, CarRadius * VFHUnitsPerMapUnit, (i32)(2.0f * VFHUnitsPerMapUnit * Car.MaxSensorRange), 180, SafetyDistance, VFHUnitsPerMapUnit);
    printf("%f seconds spent on InitVFH\n", GetSecondsElapsed(VFHStartupCounter, GetHardwareTime()));
    VFH.FollowTarget = false;
    VFH.MinSpeed = 8.0f;
    VFH.MaxSpeed = 12.0f;
    VFH.Threshold = 0.8f;
    VFH.TargetDirectionWeight = 4;
    VFH.CurrentDirectionWeight = 2;
    VFH.PreviousDirectionWeight = 1;

    vph_state VPH = {};
    InitVPH(&VPH, &Car);
    VPH.DistanceThreshold = CarRadius * 1.25f;
    VPH.MaxSpeed = 10.0f;
    VPH.SafetyCoeff = 2.0f;
    VPH.Accel = 5.0f;

    follow_gap FTG = {};
    InitFTG(&FTG, &Car);
    FTG.Threshold = 1.0f;
    FTG.scan_dist = 5.0f;
    FTG.speed_factor = 0.4f;
    FTG.MinSpeed = 5;
    FTG.MaxSpeed = 10;
    FTG.MinGapSize = 20;
    FTG.SafetyDistance = 2.0f * CarRadius;
    FTG.StopThreshold = 2 * Car.Height;

    b32 ShowHistograms = true;
    b32 CameraLocked = false;

    GLuint MyProgram = CreateVFShader(SimpleVS, SimpleFS);
    GLuint TextureProgram = CreateVFShader(TextureVS, TextureFS);
    mat4 Projection = OrthographicMatrix(0, (f32) ScreenWidth, 0, (f32) ScreenHeight, -1.0f, 100.0f);

    //Saving the map onto a texture so we don't have to waste CPU and GPU resources...
    //drawing out each individual tile in the map
    GLuint MapTexture;
    glGenTextures(1, &MapTexture);
    glBindTexture(GL_TEXTURE_2D, MapTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);

    u8* MapTexData = (u8*) malloc(3 * Grid.Width * Grid.Height * sizeof(u8));
    memset(MapTexData, 0, 3 * Grid.Width * Grid.Height);
    for(u32 Index = 0; Index < Grid.Width*Grid.Height; ++Index)
    {
        if(Grid.Data[Index] == BLOCKED_CELL)
        {
            MapTexData[3 * Index] = 0;
            MapTexData[3 * Index+1] = 0;
            MapTexData[3 * Index+2] = 0;
        }
        else
        {
            MapTexData[3 * Index] = 1;
            MapTexData[3 * Index+1] = 0;
            MapTexData[3 * Index+2] = 0;
        }
    }
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, Grid.Width, Grid.Height, 0, GL_RGB, GL_UNSIGNED_BYTE, MapTexData);

    int Running = true;
    while(Running)
    {
        u64 StartCounter = SDL_GetPerformanceCounter();
        SDL_GL_SwapWindow(Window);
        glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(MyProgram);
        SDL_Event Event;
        int PollStatus = SDL_PollEvent(&Event);
        if(PollStatus)
        {
            switch(Event.type)
            {
                case SDL_QUIT:
                    Running = false;
                break;
                case SDL_MOUSEBUTTONUP:
                {
                    int ScreenWidth, ScreenHeight;
                    SDL_GetWindowSize(Window, &ScreenWidth, &ScreenHeight);
                    SDL_MouseButtonEvent MouseState = Event.button;
                    TargetX = (f32) (MouseState.x + CameraPosition.X) / PixelsPerMapUnit;
                    TargetY = (f32) (ScreenHeight - MouseState.y + CameraPosition.Y) / PixelsPerMapUnit;
                } break;
                default:
                break;
            }
        }
        const u8* KeyStates = SDL_GetKeyboardState(0);
        Running = !(KeyStates[SDL_SCANCODE_LALT] && KeyStates[SDL_SCANCODE_F4]);

        static u32 PrevDownPress;
        static u32 PrevUpPress;
        static u32 PrevLeftPress;
        static u32 PrevRightPress;
        static u32 PrevKeyZPress;
        static u32 PrevKeyXPress;
        static u32 PrevKeyCPress;
        static u32 PrevKeyVPress;
        static u32 PrevKeyGPress;
        static u32 PrevKeyOPress;

        if(PrevDownPress && !KeyStates[SDL_SCANCODE_DOWN])
        {
            Car.MotorSpeed = -Driver.SpeedOptions[0];
        }
        if(PrevUpPress && !KeyStates[SDL_SCANCODE_UP])
        {
            Car.MotorSpeed = Driver.SpeedOptions[0];
        }
        if(PrevLeftPress && !KeyStates[SDL_SCANCODE_LEFT])
        {
            Car.TurnAngle += 0.1f;
        }
        if(PrevRightPress && !KeyStates[SDL_SCANCODE_RIGHT])
        {
            Car.TurnAngle -= 0.1f;
        }
        if(PrevKeyZPress && !KeyStates[SDL_SCANCODE_Z])
        {
            Car.TurnAngle = 0.0f;
        }
        if(PrevKeyXPress && !KeyStates[SDL_SCANCODE_X])
        {
            Car.MotorSpeed = 0;
        }
        if(PrevKeyCPress && !KeyStates[SDL_SCANCODE_C])
        {
            CameraLocked = !CameraLocked;
        }
        if(PrevKeyVPress && !KeyStates[SDL_SCANCODE_V])
        {
            UseDriver = !UseDriver;
        }
        if(PrevKeyGPress && !KeyStates[SDL_SCANCODE_G])
        {
            VFH.FollowTarget = !VFH.FollowTarget;
        }
        if(PrevKeyOPress && !KeyStates[SDL_SCANCODE_O])
        {
            ShowHistograms = !ShowHistograms;
        }

        PrevDownPress = KeyStates[SDL_SCANCODE_DOWN];
        PrevUpPress = KeyStates[SDL_SCANCODE_UP];
        PrevLeftPress = KeyStates[SDL_SCANCODE_LEFT];
        PrevRightPress = KeyStates[SDL_SCANCODE_RIGHT];
        PrevKeyZPress = KeyStates[SDL_SCANCODE_Z];
        PrevKeyXPress = KeyStates[SDL_SCANCODE_X];
        PrevKeyCPress = KeyStates[SDL_SCANCODE_C];
        PrevKeyVPress = KeyStates[SDL_SCANCODE_V];
        PrevKeyGPress = KeyStates[SDL_SCANCODE_G];
        PrevKeyOPress = KeyStates[SDL_SCANCODE_O];

        UpdateCar(&Car, Grid, SecondsPerFrame);
        SimulateScanSensor(&Car, Grid);
        f32 DirX = TargetX - Car.CenterX;
        f32 DirY = TargetY - Car.CenterY;
        VFH.TargetHeading = atan2f(DirY, DirX) - Car.Angle;
        u64 VFHStartTime = SDL_GetPerformanceCounter();
        RunDriver(&Driver, &Car, &VFH, &VPH, &FTG);
        f32 DriverTimeSpent = GetSecondsElapsed(VFHStartTime, SDL_GetPerformanceCounter());
        char DriverMessage[64];
        snprintf(DriverMessage, ArrayCount(DriverMessage), "Time spent running Driver: %f", DriverTimeSpent);
        printf("%s\n", DriverMessage);

        if(!UseDriver || Driver.StopCar)
        {
            UseDriver = false;
            Car.MotorSpeed = 0.0f;
        }

        CameraVelocity.Y += KeyStates[SDL_SCANCODE_W] ? CamSpeed * PixelsPerMapUnit : 0;
        CameraVelocity.X -= KeyStates[SDL_SCANCODE_A] ? CamSpeed * PixelsPerMapUnit : 0;
        CameraVelocity.Y -= KeyStates[SDL_SCANCODE_S] ? CamSpeed * PixelsPerMapUnit : 0;
        CameraVelocity.X += KeyStates[SDL_SCANCODE_D] ? CamSpeed * PixelsPerMapUnit : 0;

        if(CameraLocked)
        {

            CameraPosition.X = (Car.CenterX*PixelsPerMapUnit) - (f32) ScreenWidth/2;
            CameraPosition.Y = (Car.CenterY*PixelsPerMapUnit) - (f32) ScreenHeight/2;
        }
        else
        {
            CameraPosition.X += CameraVelocity.X * SecondsPerFrame;
            CameraPosition.Y += CameraVelocity.Y * SecondsPerFrame;
            CameraVelocity.X = 0;
            CameraVelocity.Y = 0;
        }

        mat4 ProjViewMat = Projection * CreateTranslation(-CameraPosition);
        GLint ProjViewLoc = glGetUniformLocation(MyProgram, "ProjView");
        glUniformMatrix4fv(ProjViewLoc, 1, GL_TRUE, ProjViewMat.M);

        //note(andrew): RotatePolygon should always be offseted by PI32/2 when...
        //the quad being rotated wants to face a direction
        BeginVertexBatching();
#if 1
        for(u32 Index = 0; Index < Grid.Width*Grid.Height; ++Index)
        {
            if(Grid.Data[Index] == BLOCKED_CELL)
            {
                ColorPolygon(0.0f, 0.0f, 0.0f, 1.0f);
            }
            else
            {
                ColorPolygon(1.0f, 1.0f, 1.0f, 1.0f);
            }
            RotatePolygon(0);
            f32 GridX = (f32) (Index % Grid.Width);
            f32 GridY = (f32) (Index / Grid.Width);
            TranslatePolygon(GridX*PixelsPerMapUnit,  GridY*PixelsPerMapUnit);
            PushQuad(PixelsPerMapUnit, PixelsPerMapUnit, true);
        }
#else
        glUseProgram(TextureProgram);
        glBindTexture(GL_TEXTURE_2D, MapTexture);
        ColorPolygon(1.0f, 1.0f, 1.0f, 1.0f);
        PushQuad(Grid.Width * PixelsPerMapUnit, Grid.Height * PixelsPerMapUnit, true);
        glUseProgram(MyProgram);
#endif
        for(i32 Count=0; Count < Car.ScanSize; ++Count)
        {
            ColorPolygon(1.0f, 0.0f, 0.0f, 1.0f);
            f32 PixelHeight = Car.ScanDistances[Count] * PixelsPerMapUnit;
            TranslatePolygon(Car.CenterX * PixelsPerMapUnit, Car.CenterY * PixelsPerMapUnit + PixelHeight/2); 
            OffsetOriginPolygon(0.0f, -PixelHeight/2);
            RotatePolygon(Car.Angle + GetAngleOfScanSample(Count, Car.ScanSize, Car.MinSensorAngle, Car.MaxSensorAngle) - PI32/2);
            PushQuad(1.0f, PixelHeight, true);
            if(Driver.AvoidanceMethod == Avoid_VPH)
            {
                ColorPolygon(0.0f, 0.0f, 1.0f, 1.0f);
                f32 PixelHeight = VPH.ModifiedDistances[Count] * PixelsPerMapUnit;
                TranslatePolygon(Car.CenterX * PixelsPerMapUnit, Car.CenterY * PixelsPerMapUnit + PixelHeight/2); 
                OffsetOriginPolygon(0.0f, -PixelHeight/2);
                RotatePolygon(Car.Angle + GetAngleOfScanSample(Count, Car.ScanSize, Car.MinSensorAngle, Car.MaxSensorAngle) - PI32/2);
                PushQuad(1.0f, PixelHeight, true);
            }
        }

        //Drawing the car
        ColorPolygon(0.0f, 1.0f, 1.0f, 1.0f);
        TranslatePolygon(Car.CenterX * PixelsPerMapUnit, Car.CenterY * PixelsPerMapUnit);
        RotatePolygon(Car.Angle - PI32/2);
        PushQuad(Car.Width * PixelsPerMapUnit, Car.Height * PixelsPerMapUnit, true);

        //Visualizing the turn angle
        ColorPolygon(0.0f, 0.0f, 1.0f, 0.5f);
        TranslatePolygon(Car.CenterX * PixelsPerMapUnit, (0.5f * Car.Height + Car.CenterY) * PixelsPerMapUnit);
        OffsetOriginPolygon(0.0f, -0.5f * Car.Height * PixelsPerMapUnit);
        RotatePolygon(Car.Angle + Car.TurnAngle - PI32/2);
        PushQuad(0.5f * Car.Width * PixelsPerMapUnit, Car.Height * PixelsPerMapUnit, true);

        f32 AngularResolution = TAU32 / (f32) VFH.AngularSectors;
        if(Driver.AvoidanceMethod == Avoid_VFH)
        {
            //Drawing the selected sector
            ColorPolygon(1.0f, 1.0f, 0.0f, 0.5f);
            TranslatePolygon(Car.CenterX * PixelsPerMapUnit, (0.5f * Car.Height + Car.CenterY) * PixelsPerMapUnit);
            OffsetOriginPolygon(0.0f, -0.5f * Car.Height * PixelsPerMapUnit);
            RotatePolygon(Car.Angle + (f32) VFH.SelectedAngle - PI32/2);
            PushQuad(0.5f * Car.Width * PixelsPerMapUnit, Car.Height * PixelsPerMapUnit, true);
        }

        //Drawing the target
        ColorPolygon(1.0f, 0.0f, 0.0f, 0.5f);
        TranslatePolygon(TargetX * PixelsPerMapUnit, TargetY * PixelsPerMapUnit);
        PushQuad(PixelsPerMapUnit, PixelsPerMapUnit, true);

        //Drawing the target sector
        if(VFH.FollowTarget)
        {
            ColorPolygon(1.0f, 0.0f, 0.0f, 0.5f);
            TranslatePolygon(Car.CenterX * PixelsPerMapUnit, (0.5f * Car.Height + Car.CenterY) * PixelsPerMapUnit);
            OffsetOriginPolygon(0.0f, -0.5f * Car.Height * PixelsPerMapUnit);
            RotatePolygon(Car.Angle + (f32) GetAngleOfSector(&VFH, VFH.TargetSector));
            PushQuad(0.5f * Car.Width * PixelsPerMapUnit, Car.Height * PixelsPerMapUnit, true);
        }

        if(Driver.AvoidanceMethod == Avoid_VFH)
        {
            for(i32 Count=0; Count < VFH.RegionDiameter*VFH.RegionDiameter; ++Count)
            {
                if(!VFH.Region[Count])
                    continue;

                ColorPolygon(1.0f, 0.0f, 0.5f, 0.5f);
                f32 RegionDistance = VFH.RegionDistances[Count];
                f32 RegionDirection = VFH.RegionDirections[Count];
                TranslatePolygon(Car.CenterX*PixelsPerMapUnit + (RegionDistance * cosf(Car.Angle + RegionDirection)) * PixelsPerVFHUnit,
                              Car.CenterY*PixelsPerMapUnit + (RegionDistance * sinf(Car.Angle + RegionDirection)) * PixelsPerVFHUnit);
                RotatePolygon(Car.Angle + PI32/2);
                PushQuad(PixelsPerVFHUnit, PixelsPerVFHUnit, true);
            }
        }

        FinishVertexBatching();

        //HUD Drawing
        glUniformMatrix4fv(ProjViewLoc, 1, GL_TRUE, Projection.M);
        BeginVertexBatching();
        if(ShowHistograms && Driver.AvoidanceMethod == Avoid_VFH)
        {
            //Drawing Primary Histogram
            {
                f32 HUDPosX = 16.0f;
                f32 HUDPosY = 368.0f;
                ColorPolygon(0.8f, 0.0f, 0.0f, 0.5f);
                f32 BorderWidth = 2.0f;
                f32 CellWidth = 6.0f;
                f32 CellHeight = 128.0f;
                f32 HUDPadding = 16.0f;
                f32 HistogramWidth = VFH.AngularSectors * (BorderWidth + CellWidth) + 2 * HUDPadding;
                f32 HistogramHeight = CellHeight + 2 * HUDPadding;
                TranslatePolygon(HUDPosX, HUDPosY);
                PushQuad(HistogramWidth, HistogramHeight, false);

                f32 MaxPrimaryValue = GetMaxVFHPrimaryValue();
                for(i32 Count=0; Count < VFH.AngularSectors; ++Count)
                {
                    ColorPolygon(0.0f, 0.0f, 0.0f, 1.0f);
                    f32 QuadHeight = CellHeight * VFH.PrimaryHistogram[Count]/MaxPrimaryValue;
                    TranslatePolygon(HUDPosX + HUDPadding + Count * (CellWidth + BorderWidth) - BorderWidth/2, HUDPosY + HUDPadding);
                    PushQuad(CellWidth+BorderWidth, QuadHeight, false);

                    ColorPolygon(1.0f, 1.0f, 1.0f, 1.0f);
                    TranslatePolygon(HUDPosX + HUDPadding + Count * (CellWidth + BorderWidth), HUDPosY + HUDPadding);
                    PushQuad(CellWidth, QuadHeight, false);
                }
            }
            //Drawing Binary Histogram
            DrawBinaryHistogram(16.0f, 192.0f, VFH.BinaryHistogram, VFH.AngularSectors, {0.0f, 0.0f, 1.0f, 0.5f});

            //Drawing Masked Histogram
            DrawBinaryHistogram(16.0f, 16.0f, VFH.MaskedHistogram, VFH.AngularSectors, {0.0f, 1.0f, 0.0f, 0.5f});
        }
        FinishVertexBatching();


        f32 ElapsedSeconds = GetSecondsElapsed(StartCounter, SDL_GetPerformanceCounter());
        if(ElapsedSeconds < SecondsPerFrame)
        {
            i32 SleepMS = (i32) (1000 * (SecondsPerFrame - ElapsedSeconds)) - 1;
            if(SleepMS > 0)
                SDL_Delay(SleepMS);
            do 
            {
                ElapsedSeconds = GetSecondsElapsed(StartCounter, SDL_GetPerformanceCounter());
            } while(ElapsedSeconds < SecondsPerFrame);
        }
        else
        {
            printf("Missed Frame\n");
        }
    }
    SDL_DestroyWindow(Window);
    SDL_Quit();

    return 0;
}
