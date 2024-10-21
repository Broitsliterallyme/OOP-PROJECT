#ifndef COLLISION_H 
#define COLLISION_H 
#include "Body2D.h"
class Collision {
public:
    bool  CheckCircletoCircle(Body2D&, Body2D&,Vector2&, float&); 
    bool  IntersectPolygons( Body2D& , Body2D& , Vector2& , float& );
    bool  IntersectPolygons( Body2D& , Body2D& ,Vector2,Vector2, Vector2& , float& );
    bool IntersectCirclePolygon( Body2D&,Body2D&, Vector2& , float& );   
    bool IntersectCirclePolygon( Body2D&,Body2D&,Vector2, Vector2& , float& ); 
    void FindContactPoints(Body2D&, Body2D&, Vector2&, Vector2&,int&);
    bool CollisionHandle(Body2D&, Body2D&, Vector2&, float&);
    bool IntersectAABB(AABB, AABB);
private:
    void ProjectCircle( Vector2& , float , Vector2 , float& , float& );
    Vector2 FindContactPointCircle(Body2D&, Body2D&);
    Vector2 FindContactPoint(Body2D&, Body2D&);
    void PointSegmentDistance(Vector2, Vector2, Vector2 ,float&, Vector2&);
    void FindContactPointsBoxBox(Body2D&, Body2D&, Vector2&, Vector2&, int&);
    Vector2  FindArithmeticMean( std::vector<Vector2>& );
    void  ProjectVertices( std::vector<Vector2>& , Vector2& , float& , float& );
    int FindClosestPointOnPolygon( Vector2& ,  std::vector<Vector2>& );

};
   
#endif  
