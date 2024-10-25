#include "Body2D.h"

Body2D::Body2D(Vector2 position, float density, float mass, float restitution, float area, bool isstatic, 
               float radius, float width, float height, ShapeType shapeType)
    : Position(position), Density(density), Mass(mass), Restitution(restitution), 
      Area(area), isStatic(isstatic), Radius(radius), Width(width), Height(height), shape(shapeType) 
{
    Velocity = {0, 0};
    Acceleration = {0, 0};
    Force = {0, 0};
    RotationalVelocity = 0;
    Rotation = 0;
    if(shapeType==ShapeType::Circle){
        Inertia = 0.5f * mass * radius * radius;
    }
    else if(shapeType==ShapeType::Box){
        Inertia = (1.0f / 12.0f) * mass * ((width * width) + (height * height));
    }
    if(isStatic){
        InvMass = 0;
        InvInteria = 0;
    }
    else{
        if(shapeType==ShapeType::Box){
            InvMass = 1.0f / mass;
            InvInteria = 1.0f /Inertia;
            StaticFriction = 0.2f;  
            DynamicFriction = 0.1f;
        }
        else if(shapeType==ShapeType::Circle){
            InvMass = 1.0f / mass;
            InvInteria = 1.0f /Inertia;
            StaticFriction = 0.2f;
            DynamicFriction = 0.2f;
        }
    }
   if(shapeType==ShapeType::Box){
    vertices.resize(4);
    transformedvertices.resize(4);
    vertices[3] = { - width / 2.0f,  - height / 2.0f};
    vertices[2] = { width / 2.0f,  - height / 2.0f};
    vertices[1] = { width / 2.0f,  height / 2.0f};
    vertices[0] = { - width / 2.0f,  height / 2.0f};
    transformedvertices=vertices;
}
 updatevertices = true;
}

Body2D::Body2D() 
    : Position({0, 0}), Density(0), Mass(0), Restitution(0), Area(0), 
      isStatic(false), Radius(0), Width(0), Height(0), shape(ShapeType::Circle) 
{}

void Body2D::CreateCircle(Vector2 position, float density, float radius, float restitution, bool isstatic, Body2D &body) {
    float area = PI * radius * radius;
    float mass = area * density;
    ShapeType shapeType = ShapeType::Circle;
    body = Body2D(position, density, mass, restitution, area, isstatic, radius, 0.0f, 0.0f, shapeType);
}

void Body2D::CreateRectangle(Vector2 position, float density, float length, float width, float restitution, bool isstatic, Body2D &body) {
    float area = length * width;
    float mass = area * density;
    ShapeType shapeType = ShapeType::Box;
    body = Body2D(position, density, mass, restitution, area, isstatic, 0.0f, width, length, shapeType);

}

void Body2D::Draw() {
    if (shape == ShapeType::Circle) {
        DrawCircleV(Position, Radius, BLUE);
    } 
    else if (shape == ShapeType::Box) {
    DrawFilledPolygon(transformedvertices,RED);
    }
    //DrawRectangleLines(aabb.min.x, aabb.min.y, aabb.max.x - aabb.min.x, aabb.max.y - aabb.min.y, GREEN);
}
void Body2D::getTransformedVertices() {
    if (updatevertices) {
        BodyTransform transform(Position, Rotation);
        transform.Transform(transform, vertices, transformedvertices);
        updatevertices = false;
    }

}
void Body2D::GetAABB(){
    if(aabbUpdate){
        float minx=std::numeric_limits<float>::max();
        float miny=std::numeric_limits<float>::max();
        float maxx=std::numeric_limits<float>::lowest();   
        float maxy=std::numeric_limits<float>::lowest();
    if(shape==ShapeType::Circle){
        minx=Position.x-Radius;
        maxx=Position.x+Radius;
        miny=Position.y-Radius;
        maxy=Position.y+Radius;
        aabb.min={minx,miny};
        aabb.max={maxx,maxy};
    }
    else if(shape==ShapeType::Box){

   for (int i = 0; i < 4; i++) {
    minx = std::min(minx, transformedvertices[i].x);
    maxx = std::max(maxx, transformedvertices[i].x);
    miny = std::min(miny, transformedvertices[i].y);
    maxy = std::max(maxy, transformedvertices[i].y);
}
aabb.min={minx,miny};
aabb.max={maxx,maxy};
}
aabbUpdate=false;
}

}

void Body2D::Move(Vector2 velocity) {
    Position.x += velocity.x;
    Position.y += velocity.y;
 
}

void Body2D::Moveto(Vector2 position) {
    Position = position;
  
}
void Body2D::Rotate(float angle) {
    Rotation += angle;
   
}
 Vector2 Body2D::getPosition() const {
    return Position;
  }
  Vector2 Body2D::getVelocity() const {
    return Velocity;
  }
    float Body2D::getRadius() const {
        return Radius;
    }
    float Body2D::getRestitution() const {
        return Restitution;
    }   
    float Body2D::getStaticFriction() const {
        return StaticFriction;
    }
    float Body2D::getDynamicFriction() const {
        return DynamicFriction;
    }
    float Body2D::getMass() const {
        return Mass;
    } 
    float Body2D::getInvMass() const {
        return InvMass;
    }
    float Body2D::getInvInertia() const {
        return InvInteria;
    }
    Vector2 Body2D::getForce() {
        return Force;
    }
    bool Body2D::isStaticBody() const {
        return isStatic;
    }
    void Body2D::addForce(Vector2 force) {
        Force = force;
    }  
    void Body2D::setVelocity(Vector2 velocity) {
        Velocity = velocity;
    }   
    float Body2D::getRotation() const {
        return Rotation;
    }
    float Body2D::getRotationalVelocity() const {
        return RotationalVelocity;
    }
    void Body2D::setRotation(float angle) {
        Rotation = angle;
    }
    void Body2D::setRotationalVelocity(float velocity) {
        RotationalVelocity = velocity;
    }
void Body2D::getVertices(std::vector<Vector2>& Ref) const {
        Ref=transformedvertices;
    }
void Body2D::step(float dt,float gravity) {
    if(!isStatic){
     Velocity = Vector2Add(Velocity, Vector2Scale(Vector2{0,gravity}, dt));    
    Position.x += Velocity.x * dt;
    Position.y += Velocity.y * dt;
    Rotation += RotationalVelocity * dt;
    }
    //Acceleration = Vector2Scale(Force, 1.0f / Mass);

    Force={0,0};
    updatevertices = true;
    aabbUpdate=true;
    GetAABB();
    getTransformedVertices();
  
}


