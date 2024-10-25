#ifndef BODY2D_H
#define BODY2D_H

#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>
#include "Polygon.h"
#include "Transform.h"
#include "AABB.h"
#include"limits"
enum class ShapeType {
    Circle,
    Box 
};

class Body2D {
private:
    Vector2 Position;
    Vector2 Velocity;
    Vector2 Acceleration;
    Vector2 Force;
    float Rotation;
    float RotationalVelocity;
    float Density;
    float Mass;
    float InvMass;
    float Inertia;
    float InvInteria;
    float Restitution;
    float StaticFriction;
    float DynamicFriction;
    float Area;
    bool isStatic;
    float Radius;
    float Width;
    float Height;
    bool updatevertices;
    std::vector<Vector2> vertices;
    std::vector<Vector2> transformedvertices;
    bool aabbUpdate;
    
    Body2D(Vector2 position, float density, float mass, float restitution, float area, bool isstatic, 
           float radius, float width, float height, ShapeType shapeType);
public:
    ShapeType shape;
    AABB aabb;
    Body2D();
    void CreateCircle(Vector2 position, float density, float radius, float restitution, bool isstatic, Body2D &body);
    void CreateRectangle(Vector2 position, float density, float length, float width, float restitution, bool isstatic, Body2D &body);
    void Draw();
    void getTransformedVertices();
    void GetAABB(); 
    void Move(Vector2);
    void Moveto(Vector2);
    void Rotate(float);
    void step(float ,float );
    Vector2 getPosition() const ;
    Vector2 getVelocity() const ;
    float getRotation() const;
    float getRotationalVelocity() const;
    void setRotation(float);
    void setRotationalVelocity(float);
    void addForce(Vector2);
    Vector2 getForce();
    void setVelocity(Vector2);
    float getRestitution() const;
    bool isStaticBody() const;
    float getStaticFriction() const;
    float getDynamicFriction() const;
    float getMass() const;
    float getInvMass() const;
    float getInvInertia() const;
    float getRadius() const;
    void getVertices(std::vector<Vector2>&) const;

    
};

#endif // BODY2D_H
