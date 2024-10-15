#include "Transform.h"


BodyTransform::BodyTransform(Vector2 position, float angle)
    : PositionX(position.x), PositionY(position.y), Sin(sinf(angle)), Cos(cosf(angle))
{}
BodyTransform::BodyTransform(float x, float y, float angle)
    : PositionX(x), PositionY(y), Sin(sinf(angle)), Cos(cosf(angle))
{}
void BodyTransform::Transform(BodyTransform Transform,std::vector<Vector2>& vertices,std::vector<Vector2>& transformedvertices) {
    for (size_t i = 0; i < vertices.size(); i++) {
        float x = vertices[i].x;
        float y = vertices[i].y;
        transformedvertices[i].x =  (x * Transform.Cos - y * Transform.Sin)+Transform.PositionX;
        transformedvertices[i].y =  (x * Transform.Sin + y * Transform.Cos)+Transform.PositionY;
    }
}