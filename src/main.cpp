#include "Engine.h"
Engine engine;
int main() {
    InitWindow(1280, 720, "Random Body2D");
    SetTargetFPS(0);
    std::vector<Vector2> points = { { -400, -300 }, { 400, -300 }, { 400, 300 }, { -400, 300 } };
    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        if(IsMouseButtonPressed(MOUSE_BUTTON_LEFT )||IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
        engine.dropbody();
        engine.camerahandle();
        engine.Update();
        engine.bodydraw();
        EndDrawing();
    }
    CloseWindow();

    return 0;
}
