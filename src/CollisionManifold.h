#ifndef COLLISION_MANIFOLD_H
#define COLLISION_MANIFOLD_H

#include "Body2D.h"
#include "raymath.h"

class CollisionManifold {
public:
    Body2D& bodyA; // Pointer to first body
    Body2D& bodyB; // Pointer to second body
    Vector2 normal; // Normal of collision
    float depth;    // Depth of collision
    Vector2 Point1;
    Vector2 Point2;
    int Count;

    CollisionManifold(Body2D& a, Body2D& b, Vector2 n, float d,Vector2 p1,Vector2 p2,int c)
        : bodyA(a), bodyB(b), normal(n), depth(d),Point1(p1),Point2(p2),Count(c) {}
};

#endif
