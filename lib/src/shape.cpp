#include "shape.hpp"

#include <limits>

#include "raymath.h"

// ====================================================================
// Implementierung der Grundkoerper (Port von phys.go, Teil 1)
// ====================================================================

namespace {
// Unendliche Masse/Traegheit fuer Waende (entspricht math.MaxFloat32).
constexpr float kInfinite = std::numeric_limits<float>::max();
} // namespace

// --- Polygon --------------------------------------------------------

void Polygon::ApplyForce(Vector2 force, float angForce) {
    Basic.Accel = Vector2Add(Basic.Accel, force);
    Basic.AngAccel += angForce;
}

void Polygon::Update() {
    Basic.Velocity = Vector2Add(Basic.Velocity, Basic.Accel);
    Basic.AngVelocity += Basic.AngAccel;

    Basic.Velocity = Vector2Scale(Basic.Velocity, 0.9995f);
    Basic.Location = Vector2Add(Basic.Location, Basic.Velocity);
    for (Vector2& vertex : Vertices) {
        vertex = Vector2Add(vertex, Basic.Velocity);
    }
    Basic.AngVelocity *= 0.9995f;
    Rotate(Basic.AngVelocity);

    Basic.Accel = Vector2{0.0f, 0.0f};
    Basic.AngAccel = 0.0f;
}

void Polygon::Draw(Color color, float thickness) const {
    const int count = static_cast<int>(Vertices.size());
    for (int i = 0; i < count; ++i) {
        DrawLineEx(Vertices[i], Vertices[(i + 1) % count], thickness, color);
    }
    DrawCircleV(Basic.Location, 5.0f, color);
}

void Polygon::Rotate(float angle) {
    for (Vector2& vertex : Vertices) {
        const Vector2 relativePos = Vector2Subtract(vertex, Basic.Location);
        const Vector2 rotatedPos = Vector2Rotate(relativePos, angle);
        vertex = Vector2Add(rotatedPos, Basic.Location);
    }
}

void Polygon::ResetPos(Vector2 delta) {
    for (Vector2& vertex : Vertices) {
        vertex = Vector2Add(vertex, delta);
    }
    Basic.Location = Vector2Add(Basic.Location, delta);
}

void Polygon::ApplyGravity(Vector2 gravity) {
    if (Basic.Mass < kInfinite) {
        if (!Basic.IsGrounded) {
            ApplyForce(gravity, 0.0f);
        } else {
            // Liegt auf: bei extrem langsamer Bewegung komplett stilllegen.
            if (Vector2Length(Basic.Velocity) < 0.5f &&
                Basic.AngVelocity < 0.1f && Basic.AngVelocity > -0.1f) {
                Basic.Velocity = Vector2{0.0f, 0.0f};
                Basic.AngVelocity = 0.0f;
            } else {
                const Vector2 dampingForce = Vector2Scale(Basic.Velocity, -0.5f);
                const float dampingAngForce = Basic.AngVelocity * -0.5f;
                ApplyForce(dampingForce, dampingAngForce);
            }
        }
    }
}

void Polygon::ClearState() {
    Basic.IsGrounded = false;
}

Polygon CreatePolygon(float x, float y, float w, float h, bool wall) {
    float mass;
    float inertia;

    if (!wall) {
        mass = w * h;
        inertia = mass * (w * w + h * h) / 2.0f;
    } else {
        mass = kInfinite;
        inertia = kInfinite;
    }

    Polygon poly;
    poly.Basic.Location = Vector2{x + w / 2.0f, y + h / 2.0f};
    poly.Basic.Mass = mass;
    poly.Basic.Inertia = inertia;
    poly.Basic.IsGrounded = false;
    poly.Vertices = {
        Vector2{x, y},
        Vector2{x + w, y},
        Vector2{x + w, y + h},
        Vector2{x, y + h},
    };
    return poly;
}

// --- Circle ---------------------------------------------------------

void Circle::ApplyForce(Vector2 force, float angForce) {
    Basic.Accel = Vector2Add(Basic.Accel, force);
    Basic.AngAccel += angForce;
}

void Circle::Update() {
    Basic.Velocity = Vector2Add(Basic.Velocity, Basic.Accel);
    Basic.AngVelocity += Basic.AngAccel;

    Basic.Velocity = Vector2Scale(Basic.Velocity, 0.9995f);
    Basic.Location = Vector2Add(Basic.Location, Basic.Velocity);
    Orientation = Vector2Add(Orientation, Basic.Velocity);

    Basic.AngVelocity *= 0.9995f;
    Rotate(Basic.AngVelocity);

    Basic.Accel = Vector2{0.0f, 0.0f};
    Basic.AngAccel = 0.0f;
}

void Circle::Draw(Color color, float thickness) const {
    DrawRing(Basic.Location, Radius - thickness, Radius, 0.0f, 360.0f, 1, color);
    DrawCircleV(Basic.Location, 3.0f, color);
    DrawLineEx(Basic.Location, Orientation, thickness, color);
}

void Circle::Rotate(float angle) {
    const Vector2 relativePos = Vector2Subtract(Orientation, Basic.Location);
    const Vector2 rotatedPos = Vector2Rotate(relativePos, angle);
    Orientation = Vector2Add(rotatedPos, Basic.Location);
}

void Circle::ResetPos(Vector2 delta) {
    Basic.Location = Vector2Add(Basic.Location, delta);
    Orientation = Vector2Add(Orientation, delta);
}

void Circle::ApplyGravity(Vector2 gravity) {
    if (Basic.Mass < kInfinite) {
        if (!Basic.IsGrounded) {
            ApplyForce(gravity, 0.0f);
        } else {
            // Aufgelagert: Geschwindigkeit sanft herunterdaempfen.
            const Vector2 dampingForce = Vector2Scale(Basic.Velocity, -0.5f);
            const float dampingAngForce = Basic.AngVelocity * -0.5f;
            ApplyForce(dampingForce, dampingAngForce);
        }
    }
}

void Circle::ClearState() {
    Basic.IsGrounded = false;
}

Circle CreateCircle(float x, float y, float r, bool wall) {
    float mass;
    float inertia;

    if (!wall) {
        mass = r * r * 2.0f;
        inertia = (r * r * r) * 100.0f;
    } else {
        mass = kInfinite;
        inertia = kInfinite;
    }

    Circle circle;
    circle.Basic.Location = Vector2{x, y};
    circle.Basic.Mass = mass;
    circle.Basic.Inertia = inertia;
    circle.Radius = r;
    circle.Orientation = Vector2{r + x, y};
    return circle;
}

// --- Dispatcher auf Shape -------------------------------------------

void Update(Shape& shape) {
    std::visit([](auto& s) { s.Update(); }, shape);
}

void Draw(const Shape& shape, Color color, float thickness) {
    std::visit([&](const auto& s) { s.Draw(color, thickness); }, shape);
}

void Rotate(Shape& shape, float angle) {
    std::visit([&](auto& s) { s.Rotate(angle); }, shape);
}

void ApplyForce(Shape& shape, Vector2 force, float angForce) {
    std::visit([&](auto& s) { s.ApplyForce(force, angForce); }, shape);
}

void ApplyGravity(Shape& shape, Vector2 gravity) {
    std::visit([&](auto& s) { s.ApplyGravity(gravity); }, shape);
}

void ResetPos(Shape& shape, Vector2 delta) {
    std::visit([&](auto& s) { s.ResetPos(delta); }, shape);
}

void ClearState(Shape& shape) {
    std::visit([](auto& s) { s.ClearState(); }, shape);
}
