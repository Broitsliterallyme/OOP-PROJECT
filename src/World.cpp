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
            if (collision.IntersectAABB(bodies[i].aabb, bodies[j].aabb)) 
                ContactPair.emplace_back(i, j); // Store pairs that may collide
            
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
            Vector2 Point1, Point2;
            int Count = 0;
            SeperateBody(body1, body2, normal, depth);
            collision.FindContactPoints(body1, body2, Point1, Point2, Count);
            CollisionManifold Manifold(body1, body2, normal, depth, Point1, Point2, Count);
            ResolveCollisionRotationFriction(Manifold);
        }
    }
}
void World::SeperateBody(Body2D& body1,Body2D& body2,Vector2 normal,float depth){
            if (body1.isStaticBody())
                body2.Move(Vector2Scale(normal, -depth));
            else if (body2.isStaticBody())
                body1.Move(Vector2Scale(normal, depth));
            else {
                body1.Move(Vector2Scale(normal, depth / 2));
                body2.Move(Vector2Scale(normal, -depth / 2));
            }
}

void World::ResolveCollision(CollisionManifold& manifold) {
    Body2D& body1 = manifold.bodyA;
    Body2D& body2 = manifold.bodyB;
    Vector2 normal = manifold.normal;
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

void World::ResolveCollisionRotation(CollisionManifold& manifold){
       Body2D& body1 = manifold.bodyA;
    Body2D& body2 = manifold.bodyB;
    Vector2 normal = manifold.normal;
    int ContactCount=manifold.Count;
    Vector2 ContactList[2]={manifold.Point1,manifold.Point2};
    Vector2 ImplseList[2]={{0,0},{0,0}};
    Vector2 RA[2]={{0,0},{0,0}};
    Vector2 RB[2]={{0,0},{0,0}};
    float e = std::min(body1.getRestitution(), body2.getRestitution());
    for(int i=0;i<ContactCount;i++){
        Vector2 ra=Vector2Subtract(ContactList[i],body1.getPosition());
        Vector2 rb=Vector2Subtract(ContactList[i],body2.getPosition());
        Vector2 raPerp={-ra.y,ra.x};
        Vector2 rbPerp={-rb.y,rb.x};
        RA[i]=ra;
        RB[i]=rb;
        Vector2 angularLinearVelocityA=Vector2Scale(raPerp,body1.getRotationalVelocity());
        Vector2 angularLinearVelocityB=Vector2Scale(rbPerp,body2.getRotationalVelocity());
        Vector2 RelativeVelocity=Vector2Subtract
                                (Vector2Add(body1.getVelocity(),angularLinearVelocityA), 
                                Vector2Add(body2.getVelocity(),angularLinearVelocityB));
        float ContactVelocity=Vector2DotProduct(RelativeVelocity,normal);
        if(ContactVelocity>0) continue;
        float raPerpDotN=Vector2DotProduct(raPerp,normal);
        float rbPerpDotN=Vector2DotProduct(rbPerp,normal);
        float Denominator=(body1.getInvMass()+body2.getInvMass())+
                     (raPerpDotN*raPerpDotN*body1.getInvInertia())+
                    (rbPerpDotN*rbPerpDotN*body2.getInvInertia());
         float j = -(1.0f + e) * ContactVelocity;
    j /= Denominator;
    Vector2 Impulse=Vector2Scale(normal,j);
    ImplseList[i]=Impulse;        
}
    for(int i=0;i<ContactCount;i++){
        Vector2 Impulse=ImplseList[i];
        Vector2 Ra=RA[i];
        Vector2 Rb=RB[i];
        Vector2 ReboundVelocity1=Vector2Add(body1.getVelocity(),Vector2Scale(Impulse,body1.getInvMass()));
        Vector2 ReboundVelocity2=Vector2Subtract(body2.getVelocity(),Vector2Scale(Impulse,body2.getInvMass()));
        body1.setVelocity(ReboundVelocity1);
        body2.setVelocity(ReboundVelocity2);
        body1.setRotationalVelocity(body1.getRotationalVelocity()+( Vector2Cross(Ra,Impulse)*body1.getInvInertia())); 
        body2.setRotationalVelocity(body2.getRotationalVelocity()-( Vector2Cross(Rb,Impulse)*body2.getInvInertia()));
    }
}

void World::ResolveCollisionRotationFriction(CollisionManifold& manifold){
       Body2D& body1 = manifold.bodyA;
    Body2D& body2 = manifold.bodyB;
    Vector2 normal = manifold.normal;
    int ContactCount=manifold.Count;
    float RollingFriction = 0.0f;
    Vector2 ContactList[2]={manifold.Point1,manifold.Point2};
    float sf=(body1.getStaticFriction()+body2.getStaticFriction())/2;
    float df=(body1.getDynamicFriction()+body2.getDynamicFriction())/2;
    for(int i=0;i<2;i++){
        ImplseList[i]={0,0};
        FrictionImpulseList[i]={0,0};
        RA[i]={0,0};
        RB[i]={0,0};
        jList[i]=0;
    }
    float e = std::min(body1.getRestitution(), body2.getRestitution());
    for(int i=0;i<ContactCount;i++){
        Vector2 ra=Vector2Subtract(ContactList[i],body1.getPosition());
        Vector2 rb=Vector2Subtract(ContactList[i],body2.getPosition());
        Vector2 raPerp={-ra.y,ra.x};
        Vector2 rbPerp={-rb.y,rb.x};
        RA[i]=ra;
        RB[i]=rb;
        Vector2 angularLinearVelocityA=Vector2Scale(raPerp,body1.getRotationalVelocity());
        Vector2 angularLinearVelocityB=Vector2Scale(rbPerp,body2.getRotationalVelocity());
        Vector2 RelativeVelocity=Vector2Subtract
                                (Vector2Add(body1.getVelocity(),angularLinearVelocityA), 
                                Vector2Add(body2.getVelocity(),angularLinearVelocityB));
        float ContactVelocity=Vector2DotProduct(RelativeVelocity,normal);
        if(ContactVelocity>0) continue;
        float raPerpDotN=Vector2DotProduct(raPerp,normal);
        float rbPerpDotN=Vector2DotProduct(rbPerp,normal);
        float Denominator=(body1.getInvMass()+body2.getInvMass())+
                     (raPerpDotN*raPerpDotN*body1.getInvInertia())+
                    (rbPerpDotN*rbPerpDotN*body2.getInvInertia());
         float j = -(1.0f + e) * ContactVelocity;
    j /= Denominator;
    Vector2 Impulse=Vector2Scale(normal,j);
    ImplseList[i]=Impulse;
    jList[i]=j;        
}
    for(int i=0;i<ContactCount;i++){
        Vector2 Impulse=ImplseList[i];
        Vector2 Ra=RA[i];
        Vector2 Rb=RB[i];
        Vector2 ReboundVelocity1=Vector2Add(body1.getVelocity(),Vector2Scale(Impulse,body1.getInvMass()));
        Vector2 ReboundVelocity2=Vector2Subtract(body2.getVelocity(),Vector2Scale(Impulse,body2.getInvMass()));
        body1.setVelocity(ReboundVelocity1);
        body2.setVelocity(ReboundVelocity2);
        body1.setRotationalVelocity(body1.getRotationalVelocity()+( Vector2Cross(Ra,Impulse)*body1.getInvInertia())); 
        body2.setRotationalVelocity(body2.getRotationalVelocity()-( Vector2Cross(Rb,Impulse)*body2.getInvInertia()));
    }
 for(int i=0;i<ContactCount;i++){
        Vector2 ra=RA[i];
        Vector2 rb=RB[i];
        Vector2 raPerp={-ra.y,ra.x};
        Vector2 rbPerp={-rb.y,rb.x};
        Vector2 angularLinearVelocityA=Vector2Scale(raPerp,body1.getRotationalVelocity());
        Vector2 angularLinearVelocityB=Vector2Scale(rbPerp,body2.getRotationalVelocity());
        Vector2 RelativeVelocity=Vector2Subtract
                                (Vector2Add(body1.getVelocity(),angularLinearVelocityA), 
                                Vector2Add(body2.getVelocity(),angularLinearVelocityB));

        float ContactVelocity=Vector2DotProduct(RelativeVelocity,normal);
        Vector2 tangent=Vector2Subtract(RelativeVelocity,Vector2Scale(normal,ContactVelocity));
        if(Vector2Equals(tangent,Vector2Zero())) continue;
        else  tangent=Vector2Normalize(tangent);
        float raPerpDotT=Vector2DotProduct(raPerp,tangent);
        float rbPerpDotT=Vector2DotProduct(rbPerp,tangent);
        float Denominator=(body1.getInvMass()+body2.getInvMass())+
                     (raPerpDotT*raPerpDotT*body1.getInvInertia())+
                    (rbPerpDotT*rbPerpDotT*body2.getInvInertia());
         float jt = -(1.0f + e) * Vector2DotProduct(RelativeVelocity,tangent);
    jt/= Denominator;

    Vector2 FrictionImpulse;
    float j=jList[i];
    if(std::abs(jt)<=sf*j){
        FrictionImpulse=Vector2Scale(tangent,jt);
    }
    else{
        FrictionImpulse=Vector2Scale(tangent,-j*df);

    }
    FrictionImpulseList[i]=FrictionImpulse;       
}
    for(int i=0;i<ContactCount;i++){
        Vector2 FrictionImpulse=FrictionImpulseList[i];
        Vector2 Ra=RA[i];
        Vector2 Rb=RB[i];
        Vector2 ReboundVelocity1=Vector2Add(body1.getVelocity(),Vector2Scale(FrictionImpulse,body1.getInvMass()));
        Vector2 ReboundVelocity2=Vector2Subtract(body2.getVelocity(),Vector2Scale(FrictionImpulse,body2.getInvMass()));
        body1.setVelocity(ReboundVelocity1);
        body2.setVelocity(ReboundVelocity2);
        body1.setRotationalVelocity(body1.getRotationalVelocity()+( Vector2Cross(Ra,FrictionImpulse)*body1.getInvInertia())); 
        body2.setRotationalVelocity(body2.getRotationalVelocity()-( Vector2Cross(Rb,FrictionImpulse)*body2.getInvInertia()));
    }
}