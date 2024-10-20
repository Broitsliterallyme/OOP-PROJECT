#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include "Body2D.h"
#include "Collision.h"
#include "CollisionManifold.h"

class World {
private:
    struct ContactPairs {
        int i;
        int j;
        ContactPairs(int x, int y) : i(x), j(y) {}
    };

    std::vector<Body2D> bodies;
    std::vector<ContactPairs> ContactPair; // Store potential contacts
    Collision collision;
    float gravity;
    int MaxIter = 128;
    int MinIter = 1;

public:
    World();
    
    void AddBody(const Body2D& body);
    Body2D& GetBody(int index);
    void RemoveBody(Body2D& body);
    std::vector<Body2D>& GetBodies();
    void Step(float dt, int iterations);
    void ResolveCollisionBasic(CollisionManifold& manifold);
    void ResolveCollisionRotation(CollisionManifold& manifold);


private:
    void SeperateBody(Body2D& ,Body2D& Bod2,Vector2 ,float );
    void BroadPhase(); 
    void NarrowPhase(); 
};

#endif 
