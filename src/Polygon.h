

#ifndef POLYGON_H  
#define POLYGON_H

#include "raylib.h"
#include <vector>

void DrawFilledPolygon( std::vector<Vector2>& points, Color color);
void DrawPolygonLine( std::vector<Vector2>& points, Color color);

#endif 
