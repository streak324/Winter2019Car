#ifndef PATH_RENDER_H
#define PATH_RENDER_H

#include "glad.h"
#include "path_common.h"

GLuint CreateGLShader(const char* ShaderSource, GLenum Type);
GLuint CreateVFShader(const char* VertexSource, const char* FragmentSource);
void BeginVertexBatching();
void TranslatePolygon(f32 X, f32 Y);
void OffsetOriginPolygon(f32 OffsetX, f32 OffsetY);
void RotatePolygon(f32 Angle);
void ColorPolygon(f32 R, f32 G, f32 B, f32 A);
void PushQuad(f32 Width, f32 Height, b32 CenterOrigin);
void PushTriangle(f32 X0, f32 Y0, f32 X1, f32 Y1, f32 X2, f32 Y2);
void PushCircle(f32 Radius);
void FinishVertexBatching();

#endif

