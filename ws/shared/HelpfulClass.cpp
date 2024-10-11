#include "HelpfulClass.h"
// MyMathUtils.cpp
#include <cmath>

double wrapTo2Pi(double angle) {
    const double TWO_PI = 2.0 * M_PI;
    while (angle >= TWO_PI) {
        angle -= TWO_PI;
    }
    while (angle < 0.0) {
        angle += TWO_PI;
    }
    return angle;
}

void MyClass::hereIsAMethod() {
    // Implementation
}