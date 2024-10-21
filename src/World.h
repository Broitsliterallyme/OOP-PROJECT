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


private:
    void BroadPhase(); // Detect potential collisions
    void NarrowPhase(); // Handle actual collision detection
    void ResolveCollision(CollisionManifold& manifold);
    void ResolveCollisionRotation(CollisionManifold& manifold);
    void SeperateBody(Body2D& ,Body2D& ,Vector2 ,float );
    float Cross(Vector2 a, Vector2 b) {
        return a.x * b.y - a.y * b.x;
    }
};

#endif // WORLD_H