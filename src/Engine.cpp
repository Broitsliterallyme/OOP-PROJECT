#include "Engine.h"
#include <string>

Engine::Engine() {
    camera.target = {0.0f, 0.0f};
    camera.offset = {0.0f, 0.0f};
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;
    Body2D body;
    body.CreateRectangle({-400.0f,-100.0f}, 1.0f,30.0f,400.0f, 0.9f, true, body);
    body.Rotate(0.2f);
    world.AddBody(body);
    body.CreateRectangle({400.0f,-100.0f}, 1.0f,30.0f,400.0f, 0.9f, true, body);
    body.Rotate(-0.2f);
    world.AddBody(body);
    body.CreateRectangle({0.0f,200.0f}, 1.0f,30.0f,1000.0f, 0.6f, true, body);
    world.AddBody(body);
}

void Engine::dropbody() {
    Body2D body;
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    body.CreateRectangle(GetScreenToWorld2D(GetMousePosition(), camera), 1.0f, 60.0f, 60.0f, 0.6f, false, body);
        world.AddBody(body);    
    }
    if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
        body.CreateCircle(GetScreenToWorld2D(GetMousePosition(), camera), 1.0f, 30.0f, 0.6f, false, body);
        world.AddBody(body);
    }
}

void Engine::bodydraw() {
    std::vector<Body2D>& bodies = world.GetBodies();

    for (Body2D& body : bodies) {
        body.Draw();
    }
     

    
    DrawText(std::to_string(bodies.size()).c_str(), 200, 200, 40, BLACK);
    DrawText(TextFormat("Frame Time: %.3f ms", GetFrameTime() * 1000.0f), 200, 250, 20, BLACK);
    DrawFPS(0, 0);
}

void Engine::camerahandle() {
    camera.offset = {GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f};
    camera.zoom += ((float)GetMouseWheelMove() * 0.05f);
    if (camera.zoom > 3.0f) camera.zoom = 3.0f;
    else if (camera.zoom < 0.1f) camera.zoom = 0.1f;
    BeginMode2D(camera);
}

void Engine::Update() {
    std::vector<Body2D>& bodies = world.GetBodies();
    float dt = GetFrameTime();
    world.Step(dt,64); 
    int index=0;
    for(Body2D& body : bodies) {
        index++;
       if(body.aabb.max.y>=400){
          world.RemoveBody(body);

      }

  
    }
}
