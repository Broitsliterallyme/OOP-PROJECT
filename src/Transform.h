
#include <raylib.h>  
#include <cmath>     
#include <vector>    
class BodyTransform
{
    
  private:
    float PositionX;
    float PositionY;
    float Sin;
    float Cos;
    public:

    BodyTransform(Vector2 , float );
    BodyTransform(float,float,float);
    BodyTransform() = default;
    void static Transform(BodyTransform,std::vector<Vector2>&,std::vector<Vector2>&);
};

