#pragma once

struct CalibrationPoint {
    int adc;
    float distancia_mm;
};

float distanciaInterpolada(double adc);