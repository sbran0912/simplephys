#pragma once

#include <cstddef>
#include <optional>
#include <vector>

// ====================================================================
// Hilfsfunktionen (Port von referenz/lib/utils.go)
//
// Zufallszahlen und ein wertbasierter Stack. Es werden bewusst keine
// Zeiger verwendet: der Stack haelt seine Elemente direkt in einem
// vector (LIFO, oberstes Element = letztes Element).
//
// Definition: utils.cpp
// ====================================================================

// RandomInt liefert eine ganze Zufallszahl aus [start, end].
int RandomInt(int start, int end);

// RandomFloat liefert eine Gleitkomma-Zufallszahl aus [start, end).
float RandomFloat(float start, float end);

// Stack<T> ist ein generischer LIFO-Stapel.
// Pop() gibt std::nullopt zurueck, wenn der Stack leer ist (entspricht
// dem Fehlerrueckgabewert im Go-Original).
template <typename T>
class Stack {
public:
    // Push legt ein Element oben ab.
    void Push(const T& value) { data_.push_back(value); }

    // Pop entnimmt das oberste Element oder nullopt, wenn leer.
    std::optional<T> Pop() {
        if (data_.empty()) {
            return std::nullopt;
        }
        const T value = data_.back();
        data_.pop_back();
        return value;
    }

    bool IsEmpty() const { return data_.empty(); }

    std::size_t Size() const { return data_.size(); }

private:
    std::vector<T> data_;
};
