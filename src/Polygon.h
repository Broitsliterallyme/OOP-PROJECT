

#ifndef POLYGON_H  
#define POLYGON_H

#include "raylib.h"
#include <vector>

void DrawFilledPolygon( std::vector<Vector2>& points, Color color);
void DrawPolygonLine( std::vector<Vector2>& points, Color color);

void DrawFilledPolygon( std::vector<Vector2>& points, Color color)
{
    if (points.size() < 3) return;

    for (int i = 1; i < (int)points.size() - 1; i++)
    {
        DrawTriangle(points[0], points[i], points[i + 1], color);
    }
}

void DrawPolygonLine( std::vector<Vector2>& points, Color color)
{
    if (points.size() < 3) return;

    for (int i = 0; i < (int)points.size(); i++)
    {
        DrawLineV(points[i], points[(i + 1) % points.size()], color);
    }
}

#endif 
