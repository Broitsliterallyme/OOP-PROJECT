#include "Collision.h"
#include "limits"
bool  Collision::CheckCircletoCircle(Body2D &body1, Body2D &body2,Vector2& normal, float& depth) {
    normal = { 0.0f, 0.0f };
    depth = 0.0f;
    Vector2 Position1=body1.getPosition();
    Vector2 Positon2=  body2.getPosition();

    Vector2 collisionVector = Vector2Subtract(Position1,Positon2);

    float distance = Vector2Length(collisionVector);
    float overlapDistance = body1.getRadius() + body2.getRadius() - distance;

    if (overlapDistance > 0) {
         normal = Vector2Normalize(collisionVector);
        depth=overlapDistance;
        return true;
        
    }
    return false;
      }
            

void Collision::ProjectVertices( std::vector<Vector2>& vertices, Vector2& axis, float& min, float& max) {
    float dot = Vector2DotProduct(vertices[0], axis);
    min = max = dot;

    for (size_t i = 1; i < vertices.size(); i++) {
        dot = Vector2DotProduct(vertices[i], axis);
        if (dot < min) min = dot;
        if (dot > max) max = dot;
    }
}

bool Collision::IntersectPolygons(Body2D& Body1, Body2D& Body2, Vector2 centerA, Vector2 centerB, Vector2& normal, float& depth) {
    std::vector<Vector2> verticesA;
    std::vector<Vector2> verticesB;
    Body1.getVertices(verticesA);
    Body2.getVertices(verticesB);

    normal = { 0.0f, 0.0f };
    depth = std::numeric_limits<float>::max();

    auto getNormalizedAxis = [](Vector2 edge) {
        return Vector2Normalize({ -edge.y, edge.x });
    };

    auto ProjectVertices = [](const std::vector<Vector2>& vertices, Vector2 axis, float& min, float& max) {
        min = Vector2DotProduct(axis, vertices[0]);
        max = min;

        for (const Vector2& vertex : vertices) {
            float projection = Vector2DotProduct(axis, vertex);
            if (projection < min) min = projection;
            if (projection > max) max = projection;
        }
    };

    for (int i = 0; i < (int)verticesA.size(); i++) {
        Vector2 va = verticesA[i];
        Vector2 vb = verticesA[(i + 1) % verticesA.size()];

        Vector2 edge = Vector2Subtract(vb, va);
        Vector2 axis = getNormalizedAxis(edge);

        float minA, maxA, minB, maxB;
        ProjectVertices(verticesA, axis, minA, maxA);
        ProjectVertices(verticesB, axis, minB, maxB);

        if (minA >= maxB || minB >= maxA) {
            return false;
        }

        float axisDepth = std::min(maxB - minA, maxA - minB);
        if (axisDepth < depth) {
            depth = axisDepth;
            normal = axis;  
        }
    }

    // Check axes of Body B
    for (int i = 0; i < (int)verticesB.size(); i++) {
        Vector2 va = verticesB[i];
        Vector2 vb = verticesB[(i + 1) % verticesB.size()];

        Vector2 edge = Vector2Subtract(vb, va);
        Vector2 axis = getNormalizedAxis(edge);

        float minA, maxA, minB, maxB;
        ProjectVertices(verticesA, axis, minA, maxA);
        ProjectVertices(verticesB, axis, minB, maxB);

        if (minA >= maxB || minB >= maxA) {
            return false;
        }

        float axisDepth = std::min(maxB - minA, maxA - minB);
        if (axisDepth < depth) {
            depth = axisDepth;
            normal = axis;  
        }
    }

    Vector2 direction = Vector2Subtract(centerA, centerB);
    if (Vector2DotProduct(direction, normal) < 0) {
        normal = Vector2Negate(normal);
    }

    return true;
}

bool Collision::IntersectPolygons( Body2D& Body1,Body2D& Body2, Vector2& normal, float& depth) {
    std::vector<Vector2> verticesA;
    std::vector<Vector2> verticesB;
    Body1.getVertices(verticesA);
    Body2.getVertices(verticesB);
    normal = { 0.0f, 0.0f };
    depth = std::numeric_limits<float>::max();
    for (int i = 0; i < (int)verticesA.size(); i++) {
        Vector2 va = verticesA[i];
        Vector2 vb = verticesA[(i + 1) % verticesA.size()];
        
        Vector2 edge = Vector2Subtract(vb, va);
        Vector2 axis = Vector2{ -edge.y, edge.x };

        float minA, maxA, minB, maxB;
        ProjectVertices(verticesA, axis, minA, maxA);
        ProjectVertices(verticesB, axis, minB, maxB);

        if (minA >= maxB || minB >= maxA) {
            return false;
        }
        float axisDepth = std::min(maxB - minA, maxA - minB);
        if (axisDepth < depth) {
            depth = axisDepth;
            normal = axis;
        }
    }

    for (int i = 0; i < (int)verticesB.size(); i++) {
        Vector2 va = verticesB[i];
        Vector2 vb = verticesB[(i + 1) % verticesB.size()];
        
        Vector2 edge = Vector2Subtract(vb, va);
        Vector2 axis = Vector2Normalize({ -edge.y, edge.x });

        float minA, maxA, minB, maxB;
        ProjectVertices(verticesA, axis, minA, maxA);
        ProjectVertices(verticesB, axis, minB, maxB);

        if (minA >= maxB || minB >= maxA) {
            return false;
        }
               float axisDepth = std::min(maxB - minA, maxA - minB);
         if (axisDepth < depth) {
            depth = axisDepth;
            normal = axis;
        }
    }


    Vector2 centerA = FindArithmeticMean(verticesA);
    Vector2 centerB = FindArithmeticMean(verticesB);

    Vector2 direction = { centerA.x - centerB.x, centerA.y - centerB.y };

    if (direction.x * normal.x + direction.y * normal.y < 0) {
        normal.x = -normal.x;
        normal.y = -normal.y;
    }

    return true;
}
Vector2 Collision::FindArithmeticMean( std::vector<Vector2>& vertices)
{
    Vector2 sum = { 0.0f, 0.0f };
    for (const Vector2& v : vertices) {
        sum.x += v.x;
        sum.y += v.y;
    }
    return { sum.x / vertices.size(), sum.y / vertices.size() };
}
void Collision::ProjectCircle(Vector2& center, float radius, Vector2 axis, float& min, float& max)
{
    Vector2 normalizedAxis = Vector2Normalize(axis);
    Vector2 radiusVector = Vector2Scale(normalizedAxis, radius);

    Vector2 p1 = Vector2Add(center, radiusVector);
    Vector2 p2 = Vector2Subtract(center, radiusVector);

    min = Vector2DotProduct(p1, axis);
    max = Vector2DotProduct(p2, axis);

    if (min > max)
    {
        std::swap(min, max);
    }
}

int Collision::FindClosestPointOnPolygon(Vector2& circleCenter,  std::vector<Vector2>& vertices)
{
    int closestIndex = 0;
    float minDistance = Vector2Distance(circleCenter, vertices[0]);

    for (int i = 1; i < vertices.size(); i++)
    {
        float distance = Vector2Distance(circleCenter, vertices[i]);
        if (distance < minDistance)
        {
            minDistance = distance;
            closestIndex = i;
        }
    }

    return closestIndex;
}

bool Collision::IntersectCirclePolygon(Body2D& Body,Body2D& Circle, Vector2& normal,float& depth)
{
     std::vector<Vector2> vertices;
    Body.getVertices(vertices);
    Vector2 circleCenter=Circle.getPosition();
    float circleRadius=Circle.getRadius();
   depth =std::numeric_limits<float>::max();

    Vector2 axis = { 0, 0 };
    float axisDepth = 0;
    float minA, maxA, minB, maxB;

    for (size_t i = 0; i < vertices.size(); i++)
    {
        Vector2 va = vertices[i];
        Vector2 vb = vertices[(i + 1) % vertices.size()];

        Vector2 edge = Vector2Subtract(vb, va);
        axis = Vector2Normalize({ -edge.y, edge.x });
        ProjectVertices(vertices, axis, minA, maxA);
        ProjectCircle(circleCenter, circleRadius, axis, minB, maxB);

        if (minA >= maxB || minB >= maxA)
        {
            return false;
        }

        axisDepth = std::min(maxB - minA, maxA - minB);

        if (axisDepth < depth)
        {
         depth = axisDepth;
         normal = axis;
        }
    }

    int closestIndex = FindClosestPointOnPolygon(circleCenter, vertices);
    Vector2 closestPoint = vertices[closestIndex];
    
    axis = Vector2Normalize(Vector2Subtract(closestPoint, circleCenter));
    ProjectVertices(vertices, axis, minA, maxA);
    ProjectCircle(circleCenter, circleRadius, axis, minB, maxB);

    if (minA >= maxB || minB >= maxA)
    {
        return false;
    }

    axisDepth = std::min(maxB - minA, maxA - minB);

    if (axisDepth <depth)
    {
       depth = axisDepth;
       normal = axis;
    }

    Vector2 polygonCenter = { 0, 0 };
    polygonCenter=FindArithmeticMean(vertices);
    Vector2 direction = Vector2Subtract(polygonCenter, circleCenter);
    if (Vector2DotProduct(direction,normal) < 0)
    {
        normal = Vector2Negate(normal);
    }

    return true;
}
bool Collision::IntersectCirclePolygon(Body2D& Body,Body2D& Circle,Vector2 polygonCenter, Vector2& normal,float& depth)
{
     std::vector<Vector2> vertices;
    Body.getVertices(vertices);
    Vector2 circleCenter=Circle.getPosition();
    float circleRadius=Circle.getRadius();
   depth =std::numeric_limits<float>::max();

    Vector2 axis = { 0, 0 };
    float axisDepth = 0;
    float minA, maxA, minB, maxB;

    for (size_t i = 0; i < vertices.size(); i++)
    {
        Vector2 va = vertices[i];
        Vector2 vb = vertices[(i + 1) % vertices.size()];

        Vector2 edge = Vector2Subtract(vb, va);
        axis = Vector2Normalize({ -edge.y, edge.x });
        ProjectVertices(vertices, axis, minA, maxA);
        ProjectCircle(circleCenter, circleRadius, axis, minB, maxB);

        if (minA >= maxB || minB >= maxA)
        {
            return false;
        }

        axisDepth = std::min(maxB - minA, maxA - minB);

        if (axisDepth < depth)
        {
         depth = axisDepth;
         normal = axis;
        }
    }

    int closestIndex = FindClosestPointOnPolygon(circleCenter, vertices);
    Vector2 closestPoint = vertices[closestIndex];
    
    axis = Vector2Normalize(Vector2Subtract(closestPoint, circleCenter));
    ProjectVertices(vertices, axis, minA, maxA);
    ProjectCircle(circleCenter, circleRadius, axis, minB, maxB);

    if (minA >= maxB || minB >= maxA)
    {
        return false;
    }

    axisDepth = std::min(maxB - minA, maxA - minB);

    if (axisDepth <depth)
    {
       depth = axisDepth;
       normal = axis;
    }

    Vector2 direction = Vector2Subtract(polygonCenter, circleCenter);
    if (Vector2DotProduct(direction,normal) < 0)
    {
        normal = Vector2Negate(normal);
    }

    return true;
}
bool Collision::CollisionHandle(Body2D& body1, Body2D& body2, Vector2& normal, float& depth) {
    normal = {0.0f, 0.0f};
    depth = 0.0f;

    if (body1.shape == ShapeType::Circle ) {
        if(body2.shape == ShapeType::Circle)
        return CheckCircletoCircle(body1, body2, normal, depth);
     else if (body2.shape == ShapeType::Box ) 
     {
        bool result = IntersectCirclePolygon(body2, body1,body2.getPosition(), normal, depth);
        normal=Vector2Negate(normal);
        return result;
     }
    }
    else if (body1.shape == ShapeType::Box) {
        if (body2.shape == ShapeType::Circle) {
            return IntersectCirclePolygon(body1, body2,body1.getPosition(), normal, depth);
        }
        else if (body2.shape == ShapeType::Box) {
            return IntersectPolygons(body1, body2,body1.getPosition(),body2.getPosition() ,normal, depth);
        }
    }

    return false;
}
void Collision::FindContactPoints(Body2D& Body1,Body2D& Body2,Vector2& Point1,Vector2& Point2,int& Count)
{

    Count = 0;
if (Body1.shape == ShapeType::Circle ) {
        if(Body2.shape == ShapeType::Circle){  
            Point1=FindContactPointCircle(Body1,Body2);
            Count=1;
        }
        else if (Body2.shape == ShapeType::Box ){  
            Point1=FindContactPoint(Body2,Body1);
           Count=1;
        }
    }
    else if (Body1.shape == ShapeType::Box) {
        if (Body2.shape == ShapeType::Circle) {
            Point1=FindContactPoint(Body1,Body2);
            Count=1;
 
        }
        else if (Body2.shape == ShapeType::Box) {
            FindContactPointsBoxBox(Body1,Body2,Point1,Point2,Count);


        }
    }
}
Vector2 Collision::FindContactPointCircle(Body2D& Body1, Body2D& Body2)
{
   Vector2 Center1=Body1.getPosition();
    Vector2 Center2=Body2.getPosition();
    float Radius1=Body1.getRadius();
    Vector2 direction = Vector2Normalize(Vector2Subtract(Center2, Center1));
    return Vector2Add(Center1,Vector2Scale(direction, Radius1));
}
bool Collision::IntersectAABB(AABB aabb1, AABB aabb2) {
    if(aabb1.max.x < aabb2.min.x || aabb1.min.x > aabb2.max.x) return false;
    if(aabb1.max.y < aabb2.min.y || aabb1.min.y > aabb2.max.y) return false;
    return true;
}

void Collision::PointSegmentDistance( Vector2 p,  Vector2 a,  Vector2 b, float& distanceSquared, Vector2& cp) {
    Vector2 ab = Vector2Subtract(b, a);  
    Vector2 ap = Vector2Subtract(p, a);  

    float proj = Vector2DotProduct(ap, ab);
    float abLenSq = Vector2LengthSqr(ab);
    float d = proj / abLenSq;

    if (d <= 0.0f) {
        cp = a;
    } else if (d >= 1.0f) {
        cp = b;
    } else {
        cp = Vector2Add(a, Vector2Scale(ab, d));
    }

    distanceSquared = Vector2LengthSqr(Vector2Subtract(p, cp));
}

// finding closest point between a circle and a polygon
Vector2 Collision::FindContactPoint( Body2D& Box, Body2D& Circle){
    std::vector<Vector2> polygonVertices;
    Box.getVertices(polygonVertices);
    Vector2 circleCenter = Circle.getPosition();
    Vector2 cp = {0.0f, 0.0f};  // init a contact point
    float minDistSq = std::numeric_limits<float>::max();

    for (size_t i = 0; i < polygonVertices.size(); i++) {
        Vector2 va = polygonVertices[i];
        Vector2 vb = polygonVertices[(i + 1) % polygonVertices.size()];

        Vector2 contact;
        float distSq;
        PointSegmentDistance(circleCenter, va, vb, distSq, contact);

        if (distSq < minDistSq) {
            minDistSq = distSq;
            cp = contact;
        }
    }
    return cp;
}


void Collision::FindContactPointsBoxBox(Body2D& Body1, Body2D& Body2,Vector2& contact1, Vector2& contact2, int& contactCount) {
    contactCount = 0;
    std::vector<Vector2> verticesA;
    std::vector<Vector2> verticesB;
    Body1.getVertices(verticesA);
    Body2.getVertices(verticesB);
    float minDistSq = std::numeric_limits<float>::max();

    for (size_t i = 0; i < verticesA.size(); i++) {
        Vector2 p = verticesA[i];

        for (size_t j = 0; j < verticesB.size(); j++) {
            Vector2 va = verticesB[j];
            Vector2 vb = verticesB[(j + 1) % verticesB.size()];

            Vector2 cp;
            float distSq;
            PointSegmentDistance(p, va, vb, distSq, cp);

            if (FloatEquals(distSq, minDistSq)) {
                if (!Vector2Equals(cp, contact1)) {
                    contact2 = cp;
                    contactCount = 2;
                }
            } else if (distSq < minDistSq) {
                minDistSq = distSq;
                contactCount = 1;
                contact1 = cp;
            }
        }
    }
    for (size_t i = 0; i < verticesB.size(); i++) {
        Vector2 p = verticesB[i];

        for (size_t j = 0; j < verticesA.size(); j++) {
            Vector2 va = verticesA[j];
            Vector2 vb = verticesA[(j + 1) % verticesA.size()];

            Vector2 cp;
            float distSq;
            PointSegmentDistance(p, va, vb, distSq, cp);

            if (FloatEquals(distSq, minDistSq)) {
                if (!Vector2Equals(cp, contact1)) {
                    contact2 = cp;
                    contactCount = 2;
                }
            } else if (distSq < minDistSq) {
                minDistSq = distSq;
                contactCount = 1;
                contact1 = cp;
            }
        }
    }
}