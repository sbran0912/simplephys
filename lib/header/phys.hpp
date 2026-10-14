#pragma once

#include <vector>

#include "raylib.h"
#include "shape.hpp"

// ====================================================================
// KOLLISION (Port von referenz/lib/phys.go)
//
// SAT-Kollisionserkennung (DetectCollision) und Impulsaufloesung
// (ResolveCollision) fuer die Kombinationen Polygon/Polygon,
// Polygon/Circle und Circle/Circle.
//
// DetectCollision liefert das Ergebnis als Wert (CollisionResult),
// ResolveCollision die vier Kraftwerte als Wert (CollisionForces).
// Es werden keine Zeiger verwendet.
//
// Definition: phys.cpp
// ====================================================================

// CollisionResult buendelt das Ergebnis einer Kollisionserkennung.
struct CollisionResult {
    bool IsColliding = false;
    Vector2 Mtv{};                 // Minimum Translation Vector
    std::vector<Vector2> Contacts; // Kontaktpunkte
};

// CollisionForces buendelt die Kraefte fuer beide beteiligte Koerper.
// ForceA/AngForceA gehoeren zum ersten Argument, ForceB/AngForceB zum
// zweiten Argument der ResolveCollision-Funktion.
struct CollisionForces {
    Vector2 ForceA{};
    float   AngForceA = 0.0f;
    Vector2 ForceB{};
    float   AngForceB = 0.0f;
};

// --- Kollisionserkennung --------------------------------------------

CollisionResult DetectCollision(const Polygon& a, const Polygon& b);
CollisionResult DetectCollision(const Polygon& poly, const Circle& circle);
CollisionResult DetectCollision(const Circle& a, const Circle& b);

// Dispatcher auf der wertbasierten Union (entspricht der Go-Typschaltung).
CollisionResult DetectCollision(const Shape& a, const Shape& b);

// --- Kollisionsaufloesung -------------------------------------------

CollisionForces ResolveCollision(Polygon& a, Polygon& b,
                                 const std::vector<Vector2>& contacts,
                                 Vector2 mtv);
CollisionForces ResolveCollision(Polygon& poly, Circle& circle,
                                 const std::vector<Vector2>& contacts,
                                 Vector2 mtv);
CollisionForces ResolveCollision(Circle& a, Circle& b, Vector2 mtv);

// Dispatcher auf der wertbasierten Union. ForceA/AngForceA gehoeren zu a,
// ForceB/AngForceB zu b.
CollisionForces ResolveCollision(Shape& a, Shape& b,
                                 const std::vector<Vector2>& contacts,
                                 Vector2 mtv);

// ApplyCollisionForces wendet die von ResolveCollision gelieferten Kraefte
// auf beide Koerper an (entspricht ApplyForce(...) im Go-Original).
void ApplyCollisionForces(Shape& a, Shape& b, const CollisionForces& forces);
