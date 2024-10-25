#ifndef ENGINE_H
#define ENGINE_H

#include "World.h"
#include "raylib.h"

class Engine {
private:
    World world;
    Camera2D camera;

public:
    Engine();
Collision collision;
    void dropbody();
    void bodydraw();
    void camerahandle();
    void Update();
};

#endif
