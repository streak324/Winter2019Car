#include "path_render.h"
#include "path_common.h"
#include "path_math.h"
#include "stdlib.h"
#define MAX_VERTICES (1UL << 16UL)

static u32 VertexCount = 0;
static b32 VertexBatching;
static b32 PolygonInit;
static const u32 VerticesPerQuad = 6;
static const u32 VerticesPerTriangle = 3;
static f32* VertexPosCurrent;
static f32* VertexPosBufferData;
static f32* VertexColorCurrent;
static f32* VertexColorBufferData;
static f32* VertexTexCurrent;
static f32* VertexTexBufferData;
static GLuint BatchVertexArray, VertexColorVBO, VertexPosVBO, VertexTexVBO;

static f32 PolygonX;
static f32 PolygonY;
static f32 PolygonAngle;
static v4 VertexColor;
static f32 VertexPosOffsetX = 0;
static f32 VertexPosOffsetY = 0;

GLuint CreateGLShader(const char* ShaderSource, GLenum ShaderType)
{
    u32 Shader = glCreateShader(ShaderType);
    glShaderSource(Shader, 1, &ShaderSource, NULL);
    glCompileShader(Shader);
    GLint Success;
    char InfoLog[512];
    glGetShaderiv(Shader, GL_COMPILE_STATUS, &Success);
    if(!Success)
    {
        glGetShaderInfoLog(Shader, 512, 0, InfoLog);
        InvalidCodePath;
    }
    return Shader;
}

GLuint  CreateVFShader(const char* VertexSource, const char* FragSource)
{
    GLuint VShader = CreateGLShader(VertexSource, GL_VERTEX_SHADER);
    GLuint FShader = CreateGLShader(FragSource, GL_FRAGMENT_SHADER);
    GLuint Program = glCreateProgram();
    glAttachShader(Program, VShader);
    glAttachShader(Program, FShader);
    glLinkProgram(Program);
    glDeleteShader(VShader);
    glDeleteShader(FShader);
    GLint Success;
    char InfoLog[512];
    glGetProgramiv(Program, GL_LINK_STATUS, &Success);
    if(!Success)
    {
        glGetProgramInfoLog(Program, 512, 0, InfoLog);
          InvalidCodePath;
    }
    return Program;
}


void FlushVertexData()
{

    glBindVertexArray(BatchVertexArray);

#if 1
    glBindBuffer(GL_ARRAY_BUFFER, VertexPosVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(f32) * VertexCount << 1, VertexPosBufferData);
    glBindBuffer(GL_ARRAY_BUFFER, VertexColorVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(f32) * VertexCount << 2, VertexColorBufferData);
    glBindBuffer(GL_ARRAY_BUFFER, VertexTexVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(f32) * VertexCount << 1, VertexTexBufferData);
#endif
    
    glDrawArrays(GL_TRIANGLES, 0, (GLsizei)(VertexCount));
    VertexCount = 0;
    VertexPosCurrent = VertexPosBufferData;
    VertexColorCurrent = VertexColorBufferData;
    VertexTexCurrent = VertexTexBufferData;

}

void BeginVertexBatching()
{
    Assert(!VertexBatching);
    if(!PolygonInit)
    {
        PolygonInit = true;

        glGenVertexArrays(1, &BatchVertexArray);
        glBindVertexArray(BatchVertexArray);
        glGenBuffers(1, &VertexPosVBO);
        glGenBuffers(1, &VertexColorVBO);
        glGenBuffers(1, &VertexTexVBO);

        u32 VertexPosSize = MAX_VERTICES * 2 * sizeof(f32);
        u32 VertexColorSize = MAX_VERTICES * 4 * sizeof(f32);
        u32 VertexTexSize = MAX_VERTICES * 2 * sizeof(f32);

#if 0
        GLbitfield FieldMap = GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT;

        // Buffer creation flags
        GLbitfield FieldCreate = FieldMap | GL_DYNAMIC_STORAGE_BIT;

        glBindBuffer(GL_ARRAY_BUFFER, VertexPosVBO);
        glBufferStorage(GL_ARRAY_BUFFER, VertexPosSize, 0, FieldCreate);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

        // We map the VBO GPU address to our client address space. 
        VertexPosBufferData = (float*)glMapBufferRange(GL_ARRAY_BUFFER, 0, VertexPosSize, FieldMap);
        Assert(VertexPosBufferData);
        VertexPosCurrent  = VertexPosBufferData;

        glBindBuffer(GL_ARRAY_BUFFER, VertexColorVBO);
        glBufferStorage(GL_ARRAY_BUFFER, VertexColorSize, 0, FieldCreate);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, 0);
        VertexColorBufferData = (float*)glMapBufferRange(GL_ARRAY_BUFFER, 0, VertexColorSize, FieldMap);
        Assert(VertexColorBufferData);
        VertexColorCurrent = VertexColorBufferData;

        glBindBuffer(GL_ARRAY_BUFFER, VertexTexVBO);
        glBufferStorage(GL_ARRAY_BUFFER, VertexTexSize, 0, FieldCreate);
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
        VertexTexBufferData = (float*)glMapBufferRange(GL_ARRAY_BUFFER, 0, VertexTexSize, FieldMap);
        Assert(VertexTexBufferData);
        VertexTexCurrent = VertexTexBufferData;
#else
        VertexPosBufferData= (f32*)malloc(VertexPosSize);
        VertexColorBufferData= (f32*)malloc(VertexColorSize);
        VertexTexBufferData = (f32*) malloc(VertexTexSize);
        VertexPosCurrent = VertexPosBufferData;
        VertexColorCurrent = VertexColorBufferData;
        VertexTexCurrent = VertexTexBufferData;

        glBindBuffer(GL_ARRAY_BUFFER, VertexPosVBO);
        glBufferData(GL_ARRAY_BUFFER, VertexPosSize, 0, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(f32), 0);

        glBindBuffer(GL_ARRAY_BUFFER, VertexColorVBO);
        glBufferData(GL_ARRAY_BUFFER, VertexColorSize, 0, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 4*sizeof(f32), 0);

        glBindBuffer(GL_ARRAY_BUFFER, VertexTexVBO);
        glBufferData(GL_ARRAY_BUFFER, VertexTexSize, 0, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 2*sizeof(f32), 0);
#endif
    }
    Assert(VertexPosCurrent == VertexPosBufferData);
    Assert(VertexColorCurrent == VertexColorBufferData);
    Assert(VertexTexCurrent == VertexTexBufferData);
    VertexBatching = true;
}

void TranslatePolygon(f32 X, f32 Y)
{
    Assert(VertexBatching);
    PolygonX = X;
    PolygonY = Y;
}

void RotatePolygon(f32 Angle)
{
    Assert(VertexBatching);
    PolygonAngle = Angle;
}

void ColorPolygon(f32 Red, f32 Green, f32 Blue, f32 Alpha)
{
    Assert(VertexBatching);
    VertexColor = {Red, Green, Blue, Alpha};
}

void OffsetOriginPolygon(f32 OffsetX, f32 OffsetY)
{
    VertexPosOffsetX = OffsetX;
    VertexPosOffsetY = OffsetY;
}

static inline void PushPosVertex(f32 X, f32 Y)
{
    VertexPosCurrent[0] = X;
    VertexPosCurrent[1] = Y; 
    VertexPosCurrent += 2;
}

void PushQuad(f32 QuadWidth, f32 QuadHeight, b32 CenterOrigin)
{
    Assert(VertexBatching);

    if(VertexCount + VerticesPerQuad > MAX_VERTICES)
    {
        FlushVertexData();
    }
    VertexCount += VerticesPerQuad;

    f32 CosVal = Cos(PolygonAngle);
    f32 SinVal = Sin(PolygonAngle);

    f32 RightX, TopY, LeftX, BottomY;

    if(CenterOrigin)
    {
        RightX = QuadWidth/2   - VertexPosOffsetX;
        TopY =   QuadHeight/2  - VertexPosOffsetY;
        LeftX =  -QuadWidth/2  - VertexPosOffsetX;
        BottomY = -QuadHeight/2 - VertexPosOffsetY;
    }
    else
    {
        RightX = -VertexPosOffsetX;
        TopY = -VertexPosOffsetY;
        LeftX = -VertexPosOffsetX + QuadWidth;
        BottomY = -VertexPosOffsetY + QuadHeight;
    }

    PushPosVertex(PolygonX + VertexPosOffsetX + RightX * CosVal - TopY * SinVal,      PolygonY + VertexPosOffsetY + TopY * CosVal + RightX * SinVal);
    PushPosVertex(PolygonX + VertexPosOffsetX + RightX * CosVal - BottomY * SinVal,   PolygonY + VertexPosOffsetY + BottomY * CosVal + RightX * SinVal);
    PushPosVertex(PolygonX + VertexPosOffsetX + LeftX * CosVal - TopY* SinVal,        PolygonY + VertexPosOffsetY + TopY* CosVal + LeftX* SinVal);
    PushPosVertex(PolygonX + VertexPosOffsetX + RightX * CosVal - BottomY * SinVal,   PolygonY + VertexPosOffsetY + BottomY * CosVal + RightX * SinVal);
    PushPosVertex(PolygonX + VertexPosOffsetX + LeftX * CosVal - BottomY * SinVal,    PolygonY + VertexPosOffsetY + BottomY * CosVal + LeftX * SinVal);
    PushPosVertex(PolygonX + VertexPosOffsetX + LeftX * CosVal - TopY * SinVal,       PolygonY + VertexPosOffsetY + TopY * CosVal + LeftX * SinVal);
    
    for(u32 Count=0; Count < VerticesPerQuad; ++Count)
    {
        VertexColorCurrent[0] = VertexColor.X;
        VertexColorCurrent[1] = VertexColor.Y;
        VertexColorCurrent[2] = VertexColor.Z;
        VertexColorCurrent[3] = VertexColor.W;
        VertexColorCurrent += 4;
    }

    VertexTexCurrent[0] = 1.0f;
    VertexTexCurrent[1] = 1.0f;
    VertexTexCurrent[2] = 1.0f;
    VertexTexCurrent[3] = 0.0f;
    VertexTexCurrent[4] = 0.0f;
    VertexTexCurrent[5] = 1.0f;
    VertexTexCurrent[6] = 1.0f;
    VertexTexCurrent[7] = 0.0f;
    VertexTexCurrent[8] = 0.0f;
    VertexTexCurrent[9] = 0.0f;
    VertexTexCurrent[10] = 0.0f;
    VertexTexCurrent[11] = 1.0f;
    VertexTexCurrent += 12;

    PolygonX= PolygonY = PolygonAngle = VertexPosOffsetX = VertexPosOffsetY = 0;
    VertexColor = {};
}

void PushTriangle(f32 X0, f32 Y0, f32 X1, f32 Y1, f32 X2, f32 Y2)
{
    Assert(VertexBatching);
    if(VertexCount + VerticesPerTriangle > MAX_VERTICES)
    {
        FlushVertexData();
    }
    VertexCount += VerticesPerTriangle;

    for(u32 Count=0; Count < 3; ++Count)
    {
        VertexColorCurrent[0] = VertexColor.X;
        VertexColorCurrent[1] = VertexColor.Y;
        VertexColorCurrent[2] = VertexColor.Z;
        VertexColorCurrent[3] = VertexColor.W;
        VertexColorCurrent += 4;
    }

    f32 CosVal = Cos(PolygonAngle);
    f32 SinVal = Sin(PolygonAngle);

    PushPosVertex(PolygonX + VertexPosOffsetX + (X0 - VertexPosOffsetX) * CosVal, PolygonY + VertexPosOffsetY + (Y0 - VertexPosOffsetY) * SinVal);
    PushPosVertex(PolygonX + VertexPosOffsetX + (X1 - VertexPosOffsetX) * CosVal, PolygonY + VertexPosOffsetY + (Y1 - VertexPosOffsetY) * SinVal);
    PushPosVertex(PolygonX + VertexPosOffsetX + (X2 - VertexPosOffsetX) * CosVal, PolygonY + VertexPosOffsetY + (Y2 - VertexPosOffsetY) * SinVal);

    PolygonX = PolygonY = PolygonAngle = VertexPosOffsetX = VertexPosOffsetY = 0;
    VertexColor = {};
}

void FinishVertexBatching()
{
    Assert(VertexBatching);
    VertexBatching = false;
    FlushVertexData();
}
