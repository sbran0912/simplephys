#include "utils.hpp"

#include <random>

using std::mt19937;
using std::random_device;
using std::uniform_int_distribution;
using std::uniform_real_distribution;

// ====================================================================
// Implementierung der Hilfsfunktionen (Port von utils.go)
// ====================================================================

int RandomInt(int start, int end) {
    static mt19937 generator{random_device{}()};
    uniform_int_distribution<int> distribution(start, end);
    return distribution(generator);
}

float RandomFloat(float start, float end) {
    static mt19937 generator{random_device{}()};
    uniform_real_distribution<float> distribution(start, end);
    return distribution(generator);
}
