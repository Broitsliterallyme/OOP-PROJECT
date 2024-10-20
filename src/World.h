#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include "Body2D.h"
#include <string>
#include "Collision.h"
#include <algorithm>
#include"CollisionManifold.h"

class World {
private:
    std::vector<Body2D> bodies;
    std::vector<CollisionManifold> Manifolds;
    Collision collision;
    float gravity;
    int MaxIter=128;
    int MinIter=1;
    size_t BodyIgnored;

public:
    World();
    std::vector<Vector2> ContactPoints1;
    std::vector<Vector2> ContactPoints2;

    void AddBody(const Body2D& body);

    Body2D& GetBody(int index);
    void RemoveBody(Body2D& body);

    std::vector<Body2D>& GetBodies();

    void Step(float dt,int iterations);

    void ResolveCollision(CollisionManifold& manifold);
};

#endif