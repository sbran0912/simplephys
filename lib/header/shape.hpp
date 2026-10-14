#pragma once

#include <variant>
#include <vector>

#include "raylib.h"

// ====================================================================
// PHYSIK-GRUNDKOERPER (Port von referenz/lib/phys.go)
//
// Ein Shape ist ein Koerper mit Masse/Traegheit, Position/Geschwindigkeit
// und Rotation. Es gibt zwei konkrete Formen: Polygon (konvex, aus
// Eckpunkten) und Circle (Kreis mit Ausrichtungslinie).
//
// Bewusst ohne Zeiger: konkrete Formen werden als Werte erzeugt und
// zurueckgegeben. Der Typ Shape ist ein std::variant, das Go-Interface
// wird also durch eine wertbasierte Union plus freie Funktionen ersetzt.
//
// Definition: shape.cpp
// ====================================================================

// BasicShape buendelt die gemeinsamen Bewegungsgroessen.
struct BasicShape {
    Vector2 Location{};             // Schwerpunkt/Schwerpunktposition
    Vector2 Velocity{};             // Geschwindigkeit
    float   AngVelocity = 0.0f;     // Winkelgeschwindigkeit
    Vector2 Accel{};                // Beschleunigung (pro Update zurueckgesetzt)
    float   AngAccel = 0.0f;        // Winkelbeschleunigung
    float   Mass = 0.0f;            // Masse (MaxFloat = unendlich/wall)
    float   Inertia = 0.0f;         // Traegheitsmoment
    bool    IsGrounded = false;     // liegt auf einer Unterlage auf
};

// Polygon: konvexe Form aus Eckpunkten (in Weltkoordinaten).
struct Polygon {
    BasicShape Basic;
    std::vector<Vector2> Vertices;

    void Update();
    void Draw(Color color, float thickness) const;
    void Rotate(float angle);
    void ApplyForce(Vector2 force, float angForce);
    void ResetPos(Vector2 delta);
    void ApplyGravity(Vector2 gravity);
    void ClearState();
};

// Circle: Kreis mit Radius; Orientation ist ein Punkt auf dem Rand und
// dient als sichtbare Rotationsmarkierung.
struct Circle {
    BasicShape Basic;
    float   Radius = 0.0f;
    Vector2 Orientation{};

    void Update();
    void Draw(Color color, float thickness) const;
    void Rotate(float angle);
    void ApplyForce(Vector2 force, float angForce);
    void ResetPos(Vector2 delta);
    void ApplyGravity(Vector2 gravity);
    void ClearState();
};

// CreatePolygon erzeugt ein achsenparalleles Rechteck als Polygon.
// wall == true -> unendliche Masse/Traegheit (beweglicher Koerper
// bleiben unbeeinflusst).
Polygon CreatePolygon(float x, float y, float w, float h, bool wall);

// CreateCircle erzeugt einen Kreis. wall == true -> unendliche Masse.
Circle CreateCircle(float x, float y, float r, bool wall);

// Shape ist die wertbasierte Union beider Formen.
using Shape = std::variant<Polygon, Circle>;

// Freie Funktionen auf Shape (Dispatcher; entsprechen den Methoden des
// Go-Interface Shape).
void Update(Shape& shape);
void Draw(const Shape& shape, Color color, float thickness);
void Rotate(Shape& shape, float angle);
void ApplyForce(Shape& shape, Vector2 force, float angForce);
void ApplyGravity(Shape& shape, Vector2 gravity);
void ResetPos(Shape& shape, Vector2 delta);
void ClearState(Shape& shape);
