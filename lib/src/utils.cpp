#include "utils.hpp"

#include <random>

// ====================================================================
// Implementierung der Hilfsfunktionen (Port von utils.go)
// ====================================================================

int RandomInt(int start, int end) {
    static std::mt19937 generator{std::random_device{}()};
    std::uniform_int_distribution<int> distribution(start, end);
    return distribution(generator);
}

float RandomFloat(float start, float end) {
    static std::mt19937 generator{std::random_device{}()};
    std::uniform_real_distribution<float> distribution(start, end);
    return distribution(generator);
}
