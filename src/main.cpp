#include "Engine.h"
Engine engine;
int main() {
    InitWindow(1280, 720, "Random Body2D");
    SetTargetFPS(0);
    std::vector<Vector2> points = { { -400, -300 }, { 400, -300 }, { 400, 300 }, { -400, 300 } };
    while (!WindowShouldClose()) {
        if(IsMouseButtonPressed(MOUSE_BUTTON_LEFT )||IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
        engine.dropbody();
        engine.Update();
        BeginDrawing();
        ClearBackground(RAYWHITE);
        engine.camerahandle();
        engine.bodydraw();
        EndDrawing();
    }
    CloseWindow();

    return 0;
}
