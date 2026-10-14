#include "physics.hpp"

#include <string>
#include <vector>

using std::string;
using std::vector;

// ====================================================================
// Beispiel: kleine Physik-Szene (Port von referenz/main.go)
//
// Boden + gestapelte Mauer + zwei Kreise. Gravitation wirkt auf alle
// Koerper, Kollisionen werden per SAT erkannt und impulsbasiert
// aufgeloest. Alles wertbasiert (kein Zeiger auf die Shapes).
//
// Start erfolgt standardmaessig im Fenstermodus (1400x700). Mit
// --fullscreen (oder -f) startet die Szene im Vollbildmodus.
// F11 schaltet zur Laufzeit zwischen Fenster und Vollbild um.
// ====================================================================

int main(int argc, char** argv) {
    const int screenWidth = 2800;
    const int screenHeight = 1400;

    bool fullscreen = false;
    for (int i = 1; i < argc; ++i) {
        const string arg = argv[i];
        if (arg == "--fullscreen" || arg == "-f") {
            fullscreen = true;
        }
    }

    SetConfigFlags(fullscreen ? (FLAG_VSYNC_HINT | FLAG_FULLSCREEN_MODE)
                              : FLAG_VSYNC_HINT);
    InitWindow(screenWidth, screenHeight, "Physics - C++");

    vector<Shape> shapes;

    // Boden (unbeweglich).
    shapes.push_back(CreatePolygon(700, 1000, 2000, 100, true));

    // Mauer aus 4 gestapelten Bloecken mit zufaelliger Breite und Hoehe.
    const float baseX = 1600.0f;
    const float baseY = 1000.0f;
    float currentY = baseY;

    for (int i = 0; i < 4; ++i) {
        const float blockW = 120.0f + RandomFloat(0.0f, 80.0f);
        const float blockH = 70.0f + RandomFloat(0.0f, 70.0f);
        const float blockX = baseX - blockW / 2.0f;

        shapes.push_back(CreatePolygon(blockX, currentY - blockH, blockW, blockH, false));
        currentY -= blockH;
    }

    // Zwei Kreise mit einmaligem Impuls nach rechts.
    Circle circle1 = CreateCircle(800, 800, 84, false);
    Circle circle2 = CreateCircle(600, 600, 70, false);
    circle1.ApplyForce(Vector2{20.0f, 0.0f}, 0.0f);
    circle2.ApplyForce(Vector2{24.0f, 0.0f}, 0.0f);
    shapes.push_back(circle1);
    shapes.push_back(circle2);

    while (!WindowShouldClose()) {
        // F11 wechselt zwischen Fenster- und Vollbildmodus.
        if (IsKeyPressed(KEY_F11)) {
            ToggleFullscreen();
        }

        BeginDrawing();
        ClearBackground(DARKBLUE);

        // Gravitation auf alle Objekte anwenden.
        for (Shape& shape : shapes) {
            ApplyGravity(shape, Vector2{0.0f, 0.4f});
            ClearState(shape);
        }

        // Kollisionen pruefen und aufloesen.
        for (size_t i = 0; i + 1 < shapes.size(); ++i) {
            for (size_t j = i + 1; j < shapes.size(); ++j) {
                const CollisionResult hit = DetectCollision(shapes[i], shapes[j]);
                if (hit.IsColliding) {
                    const CollisionForces forces =
                        ResolveCollision(shapes[i], shapes[j], hit.Contacts, hit.Mtv);
                    ApplyCollisionForces(shapes[i], shapes[j], forces);
                }
            }
        }

        // Alle Objekte aktualisieren und zeichnen.
        for (Shape& shape : shapes) {
            Update(shape);
            Draw(shape, BEIGE, 3.0f);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
