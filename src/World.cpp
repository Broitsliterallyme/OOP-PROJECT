#include "World.h"
#include "raymath.h"

World::World() : gravity(980.0f) {}

void World::AddBody(const Body2D& body) {
    bodies.push_back(body);
}

Body2D& World::GetBody(int index) {
    return bodies[index];
}

void World::RemoveBody(Body2D& body) {
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (&bodies[i] == &body) {
            bodies.erase(bodies.begin() + i);
            break;

        }
    }
}

std::vector<Body2D>& World::GetBodies() {
    return bodies;
}

void World::Step(float dt, int iterations) {
    iterations = std::max(MinIter, std::min(iterations, MaxIter));

    for (int iter = 0; iter < iterations; ++iter) {
        if (bodies.empty()) return;

        for (Body2D& body : bodies) {
            body.step(dt / iterations, gravity);
        }

        BroadPhase();  // Perform broad phase collision detection
        NarrowPhase(); // Perform narrow phase collision detection
    }
}

void World::BroadPhase() {
    ContactPair.clear();

    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            if (collision.IntersectAABB(bodies[i].aabb, bodies[j].aabb)) {
                ContactPair.emplace_back(i, j); // Store pairs that may collide
            }
        }
    }
}

void World::NarrowPhase() {
    Vector2 normal;
    float depth;

    for (const ContactPairs& pair : ContactPair) {
        Body2D& body1 = bodies[pair.i];
        Body2D& body2 = bodies[pair.j];

        if (collision.CollisionHandle(body1, body2, normal, depth)) {
            if (body1.isStaticBody())
                body2.Move(Vector2Scale(normal, -depth));
            else if (body2.isStaticBody())
                body1.Move(Vector2Scale(normal, depth));
            else {
                body1.Move(Vector2Scale(normal, depth / 2));
                body2.Move(Vector2Scale(normal, -depth / 2));
            }

            Vector2 Point1, Point2;
            int Count = 0;
            collision.FindContactPoints(body1, body2, Point1, Point2, Count);
            CollisionManifold Manifold(body1, body2, normal, depth, Point1, Point2, Count);
         ResolveCollision(Manifold);
        }
    }
}

void World::ResolveCollision(CollisionManifold& manifold) {
    Body2D& body1 = manifold.bodyA;
    Body2D& body2 = manifold.bodyB;
    Vector2 normal = manifold.normal;
    float depth = manifold.depth;

    // Only resolve if there's an overlap
    if (depth < 0) normal = Vector2Negate(normal);
    if (body1.isStaticBody() && body2.isStaticBody()) return;

    Vector2 velocity1 = body1.getVelocity();
    Vector2 velocity2 = body2.getVelocity();
    Vector2 relativeVelocity = Vector2Subtract(velocity1, velocity2);

    float velocityAlongNormal = Vector2DotProduct(relativeVelocity, normal);
    if (velocityAlongNormal > 0) return;

    float e = std::min(body1.getRestitution(), body2.getRestitution());
    float j = -(1.0f + e) * velocityAlongNormal;
    j /= (body1.getInvMass() + body2.getInvMass());

    body1.setVelocity(Vector2Add(velocity1, Vector2Scale(normal, j * body1.getInvMass())));
    body2.setVelocity(Vector2Subtract(velocity2, Vector2Scale(normal, j * body2.getInvMass())));
}
