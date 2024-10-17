#include"raylib.h"
 struct AABB{
    Vector2 max;
    Vector2 min;
     AABB(Vector2 ,Vector2 );
     AABB(float ,float ,float ,float );
     AABB(); 
     
 AABB::AABB(Vector2 Max,Vector2 Min): max(Max),min(Min)
 {}
AABB::AABB(float minx,float maxx,float miny,float maxy): max(Vector2{maxx,maxy}),min(Vector2{minx,miny})
{}
AABB::AABB():max(Vector2{0,0}),min(Vector2{0,0})
{}
};

