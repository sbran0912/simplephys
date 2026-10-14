#include "phys.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>

#include "raymath.h"

// ====================================================================
// Implementierung der Kollisionslogik (Port von phys.go, Teil 2)
//
// Hinweis zum Dispatcher: Im Go-Original liefert ResolveCollision bei
// der Reihenfolge (Circle, Polygon) die Kraefte in der (Polygon, Circle)-
// Reihenfolge zurueck. Hier wird fuer (Circle, Polygon) zurueckgetauscht,
// sodass ForceA/AngForceA immer zum ersten Argument gehoeren.
// ====================================================================

namespace {

using std::vector;

constexpr float kInfinite = std::numeric_limits<float>::max();

// Projektionsbereich eines Polygons auf einer Achse.
std::pair<float, float> ProjectPolygon(const vector<Vector2>& vertices, Vector2 axis) {
    if (vertices.empty()) {
        return {0.0f, 0.0f};
    }
    float min = Vector2DotProduct(vertices[0], axis);
    float max = min;
    for (size_t i = 1; i < vertices.size(); ++i) {
        const float proj = Vector2DotProduct(vertices[i], axis);
        if (proj < min) {
            min = proj;
        }
        if (proj > max) {
            max = proj;
        }
    }
    return {min, max};
}

// Alle Normalenachsen eines Polygons.
vector<Vector2> GetAxes(const vector<Vector2>& vertices) {
    vector<Vector2> axes;
    const size_t n = vertices.size();
    if (n == 0) {
        return axes;
    }
    for (size_t i = 0; i < n; ++i) {
        const Vector2 a = vertices[i];
        const Vector2 b = vertices[(i + 1) % n];
        const Vector2 edge = Vector2Normalize(Vector2Subtract(b, a));
        axes.push_back(Vector2{-edge.y, edge.x});
    }
    return axes;
}

// Prueft, ob ein Punkt in einem konvexen Polygon liegt.
bool PointInPolygon(Vector2 point, const vector<Vector2>& vertices) {
    const size_t n = vertices.size();
    if (n == 0) {
        return false;
    }
    for (size_t i = 0; i < n; ++i) {
        const Vector2 a = vertices[i];
        const Vector2 b = vertices[(i + 1) % n];
        const Vector2 edge = Vector2Subtract(b, a);
        const Vector2 toPoint = Vector2Subtract(point, a);
        if (Vector2DotProduct(edge, toPoint) < 0) {
            return false;
        }
    }
    return true;
}

// Ecken, die jeweils im anderen Polygon liegen.
vector<Vector2> FindContactPoints(const vector<Vector2>& verticesA,
                                  const vector<Vector2>& verticesB) {
    vector<Vector2> contacts;
    for (const Vector2& p : verticesA) {
        if (PointInPolygon(p, verticesB)) {
            contacts.push_back(p);
        }
    }
    for (const Vector2& p : verticesB) {
        if (PointInPolygon(p, verticesA)) {
            contacts.push_back(p);
        }
    }
    return contacts;
}

// Referenzkante fuer die Kontaktpunktberechnung.
std::pair<Vector2, Vector2> FindReferenceEdge(const vector<Vector2>& vertices, Vector2 normal) {
    float bestDot = -kInfinite;
    Vector2 p1{};
    Vector2 p2{};
    const size_t n = vertices.size();
    if (n == 0) {
        return {p1, p2};
    }
    for (size_t i = 0; i < n; ++i) {
        const Vector2 a = vertices[i];
        const Vector2 b = vertices[(i + 1) % n];

        const Vector2 edge = Vector2Normalize(Vector2Subtract(b, a));
        const Vector2 edgeNormal = Vector2{-edge.y, edge.x};

        const float d = Vector2DotProduct(edgeNormal, normal);
        if (d > bestDot) {
            bestDot = d;
            p1 = a;
            p2 = b;
        }
    }
    return {p1, p2};
}

// Projiziert einen Punkt auf eine Kante.
Vector2 ProjectPointOntoEdge(Vector2 p, Vector2 a, Vector2 b) {
    const Vector2 ab = Vector2Subtract(b, a);
    float t = Vector2DotProduct(Vector2Subtract(p, a), ab) / Vector2DotProduct(ab, ab);

    if (t < 0.0f) {
        t = 0.0f;
    } else if (t > 1.0f) {
        t = 1.0f;
    }

    return Vector2Add(a, Vector2Scale(ab, t));
}

// Uebertraegt Kontaktpunkte auf die Referenzkante von Polygon A.
vector<Vector2> TransferContactsToA(const vector<Vector2>& contacts,
                                    const vector<Vector2>& verticesA, Vector2 normal) {
    const auto [refEdgeStart, refEdgeEnd] = FindReferenceEdge(verticesA, normal);

    vector<Vector2> projected;
    projected.reserve(contacts.size());
    for (const Vector2& c : contacts) {
        projected.push_back(ProjectPointOntoEdge(c, refEdgeStart, refEdgeEnd));
    }
    return projected;
}

// --- Positionskorrektur je Massenverhaeltnis ------------------------

void ResetPolyPositionsBasedOnMass(Polygon& polyA, Polygon& polyB, Vector2 mtv) {
    if (polyA.Basic.Mass < kInfinite) {
        polyA.ResetPos(Vector2Scale(mtv, -0.5f));
    } else {
        polyB.ResetPos(Vector2Scale(mtv, 0.5f));
    }

    if (polyB.Basic.Mass < kInfinite) {
        polyB.ResetPos(Vector2Scale(mtv, 0.5f));
    } else {
        polyA.ResetPos(Vector2Scale(mtv, -0.5f));
    }
}

void ResetCirclePolyPositionsBasedOnMass(Polygon& poly, Circle& circle, Vector2 mtv) {
    if (poly.Basic.Mass < kInfinite) {
        poly.ResetPos(Vector2Scale(mtv, -0.5f));
    } else {
        circle.ResetPos(Vector2Scale(mtv, 0.5f));
    }

    if (circle.Basic.Mass < kInfinite) {
        circle.ResetPos(Vector2Scale(mtv, 0.5f));
    } else {
        poly.ResetPos(Vector2Scale(mtv, -0.5f));
    }
}

void ResetCirclePositionsBasedOnMass(Circle& circleA, Circle& circleB, Vector2 mtv) {
    if (circleA.Basic.Mass < kInfinite) {
        circleA.ResetPos(Vector2Scale(mtv, -0.5f));
    } else {
        circleB.ResetPos(Vector2Scale(mtv, 0.5f));
    }

    if (circleB.Basic.Mass < kInfinite) {
        circleB.ResetPos(Vector2Scale(mtv, 0.5f));
    } else {
        circleA.ResetPos(Vector2Scale(mtv, -0.5f));
    }
}

// --- Relative Vektoren am Kollisionspunkt ---------------------------

struct RelativeVectors {
    Vector2 RapPerp{};
    Vector2 RbpPerp{};
    Vector2 VgesamtA{};
    Vector2 VelocityAb{};
};

RelativeVectors CalculatePolyRelativeVectors(const Polygon& polyA, const Polygon& polyB,
                                             Vector2 collisionPoint) {
    RelativeVectors rv;
    const Vector2 rAP = Vector2Subtract(collisionPoint, polyA.Basic.Location);
    const Vector2 rBP = Vector2Subtract(collisionPoint, polyB.Basic.Location);

    rv.RapPerp = Vector2{-rAP.y, rAP.x};
    rv.RbpPerp = Vector2{-rBP.y, rBP.x};

    const Vector2 vTanA = Vector2Scale(rv.RapPerp, polyA.Basic.AngVelocity);
    const Vector2 vTanB = Vector2Scale(rv.RbpPerp, polyB.Basic.AngVelocity);
    rv.VgesamtA = Vector2Add(polyA.Basic.Velocity, vTanA);
    const Vector2 vGesamtB = Vector2Add(polyB.Basic.Velocity, vTanB);
    rv.VelocityAb = Vector2Subtract(rv.VgesamtA, vGesamtB);
    return rv;
}

RelativeVectors CalculateCirclePolyRelativeVectors(const Polygon& poly, const Circle& circle,
                                                   Vector2 collisionPoint) {
    RelativeVectors rv;
    const Vector2 rAP = Vector2Subtract(collisionPoint, poly.Basic.Location);
    const Vector2 rBP = Vector2Subtract(collisionPoint, circle.Basic.Location);

    rv.RapPerp = Vector2{-rAP.y, rAP.x};
    rv.RbpPerp = Vector2{-rBP.y, rBP.x};

    const Vector2 vTanA = Vector2Scale(rv.RapPerp, poly.Basic.AngVelocity);
    const Vector2 vTanB = Vector2Scale(rv.RbpPerp, circle.Basic.AngVelocity);
    rv.VgesamtA = Vector2Add(poly.Basic.Velocity, vTanA);
    const Vector2 vGesamtB = Vector2Add(circle.Basic.Velocity, vTanB);
    rv.VelocityAb = Vector2Subtract(rv.VgesamtA, vGesamtB);
    return rv;
}

RelativeVectors CalculateCircleRelativeVectors(const Circle& circleA, const Circle& circleB,
                                               Vector2 mtv) {
    RelativeVectors rv;
    const Vector2 rA = Vector2Scale(Vector2Normalize(mtv), -circleA.Radius);
    const Vector2 rB = Vector2Scale(Vector2Normalize(mtv), circleB.Radius);

    rv.RapPerp = Vector2{-rA.y, rA.x};
    rv.RbpPerp = Vector2{-rB.y, rB.x};

    const Vector2 vTanA = Vector2Scale(rv.RapPerp, circleA.Basic.AngVelocity);
    const Vector2 vTanB = Vector2Scale(rv.RbpPerp, circleB.Basic.AngVelocity);
    rv.VgesamtA = Vector2Add(circleA.Basic.Velocity, vTanA);
    const Vector2 vGesamtB = Vector2Add(circleB.Basic.Velocity, vTanB);
    rv.VelocityAb = Vector2Subtract(rv.VgesamtA, vGesamtB);
    return rv;
}

// --- Impulsberechnung -----------------------------------------------

float CalculatePolyImpulse(const Polygon& polyA, const Polygon& polyB, Vector2 mtv,
                           Vector2 rAP_perp, Vector2 rBP_perp, Vector2 velocity_AB, float e) {
    const float jDenominator = Vector2DotProduct(Vector2Scale(velocity_AB, -(1.0f + e)), mtv);

    const float jDivLinear = Vector2DotProduct(
        mtv, Vector2Scale(mtv, (1.0f / polyA.Basic.Mass + 1.0f / polyB.Basic.Mass)));

    const float jDivAngular =
        static_cast<float>(std::pow(Vector2DotProduct(rAP_perp, mtv), 2.0)) / polyA.Basic.Inertia +
        static_cast<float>(std::pow(Vector2DotProduct(rBP_perp, mtv), 2.0)) / polyB.Basic.Inertia;

    return jDenominator / (jDivLinear + jDivAngular);
}

float CalculateCirclePolyImpulse(const Polygon& poly, const Circle& circle, Vector2 mtv,
                                 Vector2 rAP_perp, Vector2 velocity_AB, float e) {
    const float jDenominator = Vector2DotProduct(Vector2Scale(velocity_AB, -(1.0f + e)), mtv);

    const float jDivLinear = Vector2DotProduct(
        mtv, Vector2Scale(mtv, (1.0f / poly.Basic.Mass + 1.0f / circle.Basic.Mass)));

    // nur fuer box
    const float jDivAngular =
        static_cast<float>(std::pow(Vector2DotProduct(rAP_perp, mtv), 2.0)) / poly.Basic.Inertia;

    return jDenominator / (jDivLinear + jDivAngular);
}

float CalculateCircleImpulse(const Circle& circleA, const Circle& circleB, Vector2 mtv,
                             Vector2 velocity_AB, float e) {
    const float jDenominator = Vector2DotProduct(Vector2Scale(velocity_AB, -(1.0f + e)), mtv);

    const float jDivLinear = Vector2DotProduct(
        mtv, Vector2Scale(mtv, (1.0f / circleA.Basic.Mass + 1.0f / circleB.Basic.Mass)));

    return jDenominator / jDivLinear;
}

Vector2 CalculateFrictionVector(Vector2 mtv, Vector2 velocity_AB) {
    Vector2 t = Vector2{-mtv.y, mtv.x};
    const float tScalarprodukt = Vector2DotProduct(velocity_AB, t);
    t = Vector2Normalize(Vector2Scale(t, tScalarprodukt));
    return t;
}

CollisionForces CalculatePolyCollisionForces(const Polygon& polyA, const Polygon& polyB, Vector2 mtv,
                                             Vector2 rAP_perp, Vector2 rBP_perp,
                                             Vector2 velocity_AB) {
    const float e = 0.4f;
    const Vector2 t = CalculateFrictionVector(mtv, velocity_AB);
    const float friction = -0.15f;

    const float j = CalculatePolyImpulse(polyA, polyB, mtv, rAP_perp, rBP_perp, velocity_AB, e);

    CollisionForces forces;
    forces.ForceA = Vector2Add(Vector2Scale(mtv, (j / polyA.Basic.Mass)),
                               Vector2Scale(t, (friction * -j / polyA.Basic.Mass)));
    forces.AngForceA = Vector2DotProduct(
        rAP_perp, Vector2Add(Vector2Scale(mtv, j / polyA.Basic.Inertia),
                             Vector2Scale(t, friction * -j / polyA.Basic.Inertia)));

    forces.ForceB = Vector2Add(Vector2Scale(mtv, (-j / polyB.Basic.Mass)),
                               Vector2Scale(t, (friction * j / polyB.Basic.Mass)));
    forces.AngForceB = Vector2DotProduct(
        rBP_perp, Vector2Add(Vector2Scale(mtv, -j / polyB.Basic.Inertia),
                             Vector2Scale(t, friction * j / polyB.Basic.Inertia)));
    return forces;
}

CollisionForces CalculateCirclePolyCollisionForces(const Polygon& poly, const Circle& circle,
                                                   Vector2 mtv, Vector2 rAP_perp, Vector2 rBP_perp,
                                                   Vector2 velocity_AB) {
    const float e = 0.3f;
    const Vector2 t = CalculateFrictionVector(mtv, velocity_AB);
    const float friction = -0.15f;

    const float j = CalculateCirclePolyImpulse(poly, circle, mtv, rAP_perp, velocity_AB, e);

    CollisionForces forces;
    forces.ForceA = Vector2Add(Vector2Scale(mtv, (j / poly.Basic.Mass)),
                               Vector2Scale(t, (friction * -j / poly.Basic.Mass)));
    forces.AngForceA = Vector2DotProduct(
        rAP_perp, Vector2Add(Vector2Scale(mtv, j / poly.Basic.Inertia),
                             Vector2Scale(t, friction * -j / poly.Basic.Inertia)));

    forces.ForceB = Vector2Add(Vector2Scale(mtv, (-j / circle.Basic.Mass)),
                               Vector2Scale(t, (friction * j / circle.Basic.Mass)));
    forces.AngForceB =
        Vector2DotProduct(rBP_perp, Vector2Scale(t, friction * j / circle.Basic.Inertia));
    return forces;
}

CollisionForces CalculateCircleCollisionForces(const Circle& circleA, const Circle& circleB,
                                               Vector2 mtv, Vector2 rAP_perp, Vector2 rBP_perp,
                                               Vector2 velocity_AB) {
    const float e = 0.3f;
    const Vector2 t = CalculateFrictionVector(mtv, velocity_AB);
    const float friction = -0.15f;

    const float j = CalculateCircleImpulse(circleA, circleB, mtv, velocity_AB, e);

    CollisionForces forces;
    forces.ForceA = Vector2Add(Vector2Scale(mtv, (j / circleA.Basic.Mass)),
                               Vector2Scale(t, (friction * j / circleA.Basic.Mass)));
    forces.AngForceA =
        Vector2DotProduct(rAP_perp, Vector2Scale(t, friction * j / circleA.Basic.Inertia));

    forces.ForceB = Vector2Add(Vector2Scale(mtv, (-j / circleB.Basic.Mass)),
                               Vector2Scale(t, (friction * -j / circleB.Basic.Mass)));
    forces.AngForceB =
        Vector2DotProduct(rBP_perp, Vector2Scale(t, friction * -j / circleB.Basic.Inertia));
    return forces;
}

// --- Zustandspruefungen ---------------------------------------------

// Liegt der Schwerpunkt ueber der Flaeche zwischen den aeusseren Kontakten?
bool IsCenterOfMassSupported(Vector2 location, const vector<Vector2>& contacts, Vector2 normal) {
    if (contacts.size() < 2) {
        return false;
    }

    const Vector2 tangent = Vector2{-normal.y, normal.x};
    const float comProj = Vector2DotProduct(location, tangent);

    float minProj = kInfinite;
    float maxProj = -kInfinite;

    for (const Vector2& cp : contacts) {
        const float proj = Vector2DotProduct(cp, tangent);
        if (proj < minProj) {
            minProj = proj;
        }
        if (proj > maxProj) {
            maxProj = proj;
        }
    }

    const float epsilon = 0.5f;
    return comProj >= (minProj - epsilon) && comProj <= (maxProj + epsilon);
}

// Liegt das Polygon beinahe horizontal?
bool IsHorizontal(Vector2 mtv, Vector2 gravity) {
    const float result =
        Vector2DotProduct(mtv, gravity) / (Vector2Length(mtv) * Vector2Length(gravity));
    return result > 0.9f || result < -0.9f;
}

bool IsCircleGrounded(float circleAngVelocity, Vector2 velocity_AB, Vector2 mtv) {
    if (Vector2Length(velocity_AB) > 0.5f) {
        return false;
    }

    if (std::abs(circleAngVelocity) > 0.1f) {
        return false;
    }

    const Vector2 gravity = Vector2{0.0f, 1.0f};
    const float dot = Vector2DotProduct(mtv, gravity);
    const float lenMtv = Vector2Length(mtv);
    const float lenGravity = Vector2Length(gravity);

    if (lenMtv == 0.0f || lenGravity == 0.0f) {
        return false;
    }

    return dot / (lenMtv * lenGravity) < -0.9f;
}

// Liegt das Polygon A ueber B? (mtv zeigt von A nach B)
Polygon& FindPolyTop(Polygon& polyA, Polygon& polyB, Vector2 mtv) {
    if (Vector2DotProduct(mtv, Vector2{0.0f, 1.0f}) > 0.0f) {
        return polyA;
    }
    return polyB;
}

bool IsPolyGrounded(const vector<Vector2>& contacts, Vector2 vGesamtA, Vector2 velocity_AB,
                    Vector2 mtv) {
    return contacts.size() > 1 && Vector2Length(velocity_AB) < 1.5f &&
           Vector2Length(vGesamtA) < 1.5f && IsHorizontal(mtv, Vector2{0.0f, 1.0f});
}

// Im Go-Original definiert, dort aber nicht aufgerufen.
[[maybe_unused]] bool CheckContactStability(const Polygon& poly,
                                            const vector<Vector2>& contacts, Vector2 mtv) {
    Vector2 normal = Vector2Scale(mtv, -1.0f);
    const auto [edgeStart, edgeEnd] = FindReferenceEdge(poly.Vertices, normal);
    const float edgeLength = Vector2Distance(edgeStart, edgeEnd);
    const float contactDistance = Vector2Distance(contacts[0], contacts[1]);

    float ratio = 0.0f;
    if (edgeLength > 0.0f) {
        ratio = contactDistance / edgeLength;
    }
    return ratio > 0.3f;
}

// Im Go-Original definiert, dort aber nicht aufgerufen.
[[maybe_unused]] void UpdatePolyGroundedMass(Polygon& polyA, Polygon& polyB,
                                             const vector<Vector2>& contacts, Vector2 velocityAB) {
    if (contacts.size() > 1 && Vector2Length(velocityAB) < 2.0f) {
        if (polyB.Basic.Mass == kInfinite && polyB.Basic.Inertia == kInfinite) {
            polyA.Basic.IsGrounded = true;
        } else {
            polyA.Basic.IsGrounded = false;
        }

        if (polyA.Basic.Mass == kInfinite && polyA.Basic.Inertia == kInfinite) {
            polyB.Basic.IsGrounded = true;
        } else {
            polyB.Basic.IsGrounded = false;
        }
    } else {
        polyA.Basic.IsGrounded = false;
        polyB.Basic.IsGrounded = false;
    }
}

} // namespace

// ====================================================================
// Oeffentliche Kollisionserkennung
// ====================================================================

CollisionResult DetectCollision(const Polygon& polyA, const Polygon& polyB) {
    CollisionResult result;

    float smallestOverlap = kInfinite;
    Vector2 smallestAxis{};

    vector<Vector2> axes = GetAxes(polyA.Vertices);
    const vector<Vector2> axesB = GetAxes(polyB.Vertices);
    axes.insert(axes.end(), axesB.begin(), axesB.end());

    for (const Vector2& axis : axes) {
        const auto [minA, maxA] = ProjectPolygon(polyA.Vertices, axis);
        const auto [minB, maxB] = ProjectPolygon(polyB.Vertices, axis);

        const float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        if (overlap <= 0.0f) {
            return CollisionResult{};
        }

        if (overlap < smallestOverlap) {
            smallestOverlap = overlap;
            smallestAxis = axis;

            const Vector2 dir = Vector2Subtract(polyB.Basic.Location, polyA.Basic.Location);
            if (Vector2DotProduct(dir, smallestAxis) < 0.0f) {
                smallestAxis = Vector2Scale(smallestAxis, -1.0f);
            }
        }
    }

    result.Mtv = Vector2Scale(smallestAxis, smallestOverlap);

    const vector<Vector2> rawContacts = FindContactPoints(polyA.Vertices, polyB.Vertices);
    // smallestAxis ist bereits normalisiert.
    result.Contacts =
        TransferContactsToA(rawContacts, polyA.Vertices, Vector2Scale(smallestAxis, -1.0f));

    if (!result.Contacts.empty()) {
        result.IsColliding = true;
        return result;
    }
    return CollisionResult{};
}

CollisionResult DetectCollision(const Polygon& poly, const Circle& circle) {
    CollisionResult result;

    const size_t n = poly.Vertices.size();
    float closestDistSq = kInfinite;
    Vector2 closestPoint{};
    bool inside = true;

    for (size_t i = 0; i < n; ++i) {
        const Vector2 a = poly.Vertices[i];
        const Vector2 b = poly.Vertices[(i + 1) % n];
        const Vector2 edge = Vector2Subtract(b, a);
        const Vector2 toCircle = Vector2Subtract(circle.Basic.Location, a);

        const Vector2 normal = Vector2{-edge.y, edge.x};
        if (Vector2DotProduct(normal, toCircle) < 0.0f) {
            inside = false;
        }

        const float edgeLenSq = Vector2DotProduct(edge, edge);
        const float t = Vector2DotProduct(toCircle, edge) / edgeLenSq;
        Vector2 currentClosest{};
        if (t <= 0.0f) {
            currentClosest = a;
        } else if (t >= 1.0f) {
            currentClosest = b;
        } else {
            currentClosest = Vector2Add(a, Vector2Scale(edge, t));
        }

        const Vector2 diff = Vector2Subtract(circle.Basic.Location, currentClosest);
        const float distSq = Vector2DotProduct(diff, diff);

        if (distSq < closestDistSq) {
            closestDistSq = distSq;
            closestPoint = currentClosest;
        }
    }

    if (inside || closestDistSq <= circle.Radius * circle.Radius) {
        Vector2 axis{};
        float overlap = 0.0f;

        if (inside) {
            float minOverlap = kInfinite;
            for (size_t i = 0; i < n; ++i) {
                const Vector2 a = poly.Vertices[i];
                const Vector2 b = poly.Vertices[(i + 1) % n];
                const Vector2 edge = Vector2Subtract(b, a);
                const Vector2 normal = Vector2Normalize(Vector2{-edge.y, edge.x});
                const Vector2 toCircle = Vector2Subtract(circle.Basic.Location, a);

                const float dist = Vector2DotProduct(normal, toCircle);
                const float overlapVal = dist + circle.Radius;
                if (overlapVal < minOverlap) {
                    minOverlap = overlapVal;
                    axis = Vector2Scale(normal, -1.0f); // nach aussen schieben
                }
            }
            overlap = minOverlap;
        } else {
            const float dist = std::sqrt(closestDistSq);
            overlap = circle.Radius - dist;
            if (dist > 1e-6f) {
                axis = Vector2Scale(Vector2Subtract(circle.Basic.Location, closestPoint), 1.0f / dist);
            } else {
                axis = Vector2{0.0f, -1.0f};
            }
        }

        result.IsColliding = true;
        result.Mtv = Vector2Scale(axis, overlap);
        result.Contacts.push_back(closestPoint);
        return result;
    }

    return CollisionResult{};
}

CollisionResult DetectCollision(const Circle& circleA, const Circle& circleB) {
    CollisionResult result;

    const Vector2 collisionLine =
        Vector2Subtract(circleA.Basic.Location, circleB.Basic.Location);
    const float dist = Vector2Length(collisionLine);
    const float radiusTotal = circleA.Radius + circleB.Radius;
    if (dist < radiusTotal) {
        const float overlap = dist - radiusTotal;
        result.IsColliding = true;
        result.Mtv = Vector2Scale(Vector2Normalize(collisionLine), overlap);
        return result;
    }
    return CollisionResult{};
}

CollisionResult DetectCollision(const Shape& a, const Shape& b) {
    return std::visit(
        [](const auto& shapeA, const auto& shapeB) -> CollisionResult {
            using A = std::decay_t<decltype(shapeA)>;
            using B = std::decay_t<decltype(shapeB)>;
            if constexpr (std::is_same_v<A, Polygon> && std::is_same_v<B, Polygon>) {
                return DetectCollision(shapeA, shapeB);
            } else if constexpr (std::is_same_v<A, Polygon> && std::is_same_v<B, Circle>) {
                return DetectCollision(shapeA, shapeB);
            } else if constexpr (std::is_same_v<A, Circle> && std::is_same_v<B, Polygon>) {
                return DetectCollision(shapeB, shapeA);
            } else {
                return DetectCollision(shapeA, shapeB);
            }
        },
        a, b);
}

// ====================================================================
// Oeffentliche Kollisionsaufloesung
// ====================================================================

CollisionForces ResolveCollision(Polygon& polyA, Polygon& polyB,
                                 const vector<Vector2>& contacts, Vector2 mtv) {
    // Statischer Slop: erlaubt 0.5 Pixel Ueberlappung.
    const float slop = 0.5f;
    const float mtvLength = Vector2Length(mtv);

    if (mtvLength > slop) {
        const Vector2 mtvCorrection = Vector2Scale(Vector2Normalize(mtv), mtvLength - slop);
        ResetPolyPositionsBasedOnMass(polyA, polyB, mtvCorrection);
    }

    mtv = Vector2Normalize(mtv);

    Vector2 centerCollisionPoint = contacts[0];
    if (contacts.size() > 1) {
        centerCollisionPoint = Vector2Scale(Vector2Add(contacts[0], contacts[1]), 0.5f);
    }

    // Werte nur fuer den Grounded-Check berechnen.
    const RelativeVectors rvCenter = CalculatePolyRelativeVectors(polyA, polyB, centerCollisionPoint);

    Polygon& polyTop = FindPolyTop(polyA, polyB, mtv);
    if (IsPolyGrounded(contacts, rvCenter.VgesamtA, rvCenter.VelocityAb, mtv)) {
        if (IsCenterOfMassSupported(polyTop.Basic.Location, contacts, mtv)) {
            polyTop.Basic.IsGrounded = true;
        } else {
            polyTop.Basic.IsGrounded = false;
        }
    } else {
        polyTop.Basic.IsGrounded = false;
    }

    // Impulse fuer jeden Kontaktpunkt einzeln berechnen.
    Vector2 sumForceA{};
    Vector2 sumForceB{};
    float sumAngForceA = 0.0f;
    float sumAngForceB = 0.0f;
    float appliedContacts = 0.0f;

    for (const Vector2& cp : contacts) {
        const RelativeVectors rv = CalculatePolyRelativeVectors(polyA, polyB, cp);

        if (Vector2DotProduct(rv.VelocityAb, Vector2Scale(mtv, -1.0f)) < 0.0f) {
            const CollisionForces f = CalculatePolyCollisionForces(
                polyA, polyB, mtv, rv.RapPerp, rv.RbpPerp, rv.VelocityAb);
            sumForceA = Vector2Add(sumForceA, f.ForceA);
            sumAngForceA += f.AngForceA;
            sumForceB = Vector2Add(sumForceB, f.ForceB);
            sumAngForceB += f.AngForceB;
            appliedContacts += 1.0f;
        }
    }

    CollisionForces forces;
    if (appliedContacts > 0.0f) {
        forces.ForceA = Vector2Scale(sumForceA, 1.0f / appliedContacts);
        forces.AngForceA = sumAngForceA / appliedContacts;
        forces.ForceB = Vector2Scale(sumForceB, 1.0f / appliedContacts);
        forces.AngForceB = sumAngForceB / appliedContacts;
    }
    return forces;
}

CollisionForces ResolveCollision(Polygon& poly, Circle& circle, const vector<Vector2>& contacts,
                                 Vector2 mtv) {
    const float slop = 0.5f;
    const float mtvLength = Vector2Length(mtv);

    if (mtvLength > slop) {
        const Vector2 mtvCorrection = Vector2Scale(Vector2Normalize(mtv), mtvLength - slop);
        ResetCirclePolyPositionsBasedOnMass(poly, circle, mtvCorrection);
    }

    mtv = Vector2Normalize(mtv);

    // Es gibt nur einen Kontaktpunkt.
    const Vector2 collisionPoint = contacts[0];

    const RelativeVectors rv = CalculateCirclePolyRelativeVectors(poly, circle, collisionPoint);

    // mtv zeigt von poly nach circle.
    circle.Basic.IsGrounded =
        IsCircleGrounded(circle.Basic.AngVelocity, Vector2Scale(rv.VelocityAb, -1.0f),
                         Vector2Scale(mtv, -1.0f));

    CollisionForces forces;
    if (Vector2DotProduct(rv.VelocityAb, Vector2Scale(mtv, -1.0f)) < 0.0f) {
        forces = CalculateCirclePolyCollisionForces(poly, circle, mtv, rv.RapPerp, rv.RbpPerp,
                                                    rv.VelocityAb);
    }
    return forces;
}

CollisionForces ResolveCollision(Circle& circleA, Circle& circleB, Vector2 mtv) {
    const float slop = 0.5f;
    const float mtvLength = Vector2Length(mtv);

    if (mtvLength > slop) {
        const Vector2 mtvCorrection = Vector2Scale(Vector2Normalize(mtv), mtvLength - slop);
        ResetCirclePositionsBasedOnMass(circleA, circleB, mtvCorrection);
    }

    mtv = Vector2Normalize(mtv);

    const RelativeVectors rv = CalculateCircleRelativeVectors(circleA, circleB, mtv);

    circleA.Basic.IsGrounded = IsCircleGrounded(circleA.Basic.AngVelocity, rv.VelocityAb, mtv);
    circleB.Basic.IsGrounded =
        IsCircleGrounded(circleB.Basic.AngVelocity, Vector2Scale(rv.VelocityAb, -1.0f),
                         Vector2Scale(mtv, -1.0f));

    CollisionForces forces;
    if (Vector2DotProduct(rv.VelocityAb, Vector2Scale(mtv, -1.0f)) < 0.0f) {
        forces = CalculateCircleCollisionForces(circleA, circleB, mtv, rv.RapPerp, rv.RbpPerp,
                                                rv.VelocityAb);
    }
    return forces;
}

CollisionForces ResolveCollision(Shape& a, Shape& b, const vector<Vector2>& contacts, Vector2 mtv) {
    return std::visit(
        [&](auto& shapeA, auto& shapeB) -> CollisionForces {
            using A = std::decay_t<decltype(shapeA)>;
            using B = std::decay_t<decltype(shapeB)>;
            if constexpr (std::is_same_v<A, Polygon> && std::is_same_v<B, Polygon>) {
                return ResolveCollision(shapeA, shapeB, contacts, mtv);
            } else if constexpr (std::is_same_v<A, Polygon> && std::is_same_v<B, Circle>) {
                return ResolveCollision(shapeA, shapeB, contacts, mtv);
            } else if constexpr (std::is_same_v<A, Circle> && std::is_same_v<B, Polygon>) {
                CollisionForces forces = ResolveCollision(shapeB, shapeA, contacts, mtv);
                std::swap(forces.ForceA, forces.ForceB);
                std::swap(forces.AngForceA, forces.AngForceB);
                return forces;
            } else {
                return ResolveCollision(shapeA, shapeB, mtv);
            }
        },
        a, b);
}

void ApplyCollisionForces(Shape& a, Shape& b, const CollisionForces& forces) {
    ApplyForce(a, forces.ForceA, forces.AngForceA);
    ApplyForce(b, forces.ForceB, forces.AngForceB);
}
