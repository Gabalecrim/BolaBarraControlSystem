#include <Arduino.h>
#include "sensor.h"

CalibrationPoint tabela[] = {
  {3400, 3},
  {3000, 4},
  {2500, 5},
  {2200, 6},
  {1900, 7},
  {1650, 8},
  {1450, 9},
  {1290, 10},
  {1100, 11},
  {1000, 12},
  {920, 13},
  {880, 14},
  {790, 15},
  {720, 16},
  {630, 17},
  {590, 18},
  {450, 19},
  {400, 20},
  {360, 21},
  {300, 22},
  {220, 23},
  {200, 24},
  {150, 25}
};

const int NUM_PONTOS = sizeof(tabela) / sizeof(tabela[0]);

float distanciaInterpolada(double adc) {
  if (adc >= tabela[0].adc)
    return tabela[0].distancia_mm;

  if (adc <= tabela[NUM_PONTOS - 1].adc)
    return tabela[NUM_PONTOS - 1].distancia_mm;

  for (int i = 0; i < NUM_PONTOS - 1; i++)
  {
    if (adc <= tabela[i].adc && adc >= tabela[i + 1].adc)
    {
      float x1 = tabela[i].adc;
      float x2 = tabela[i + 1].adc;
      float y1 = tabela[i].distancia_mm;
      float y2 = tabela[i + 1].distancia_mm;

      return y1 + (adc - x1) * (y2 - y1) / (x2 - x1);
    }
  }

  return 0;
}