# Ball and Beam PID Controller

Projeto de controle PID aplicado ao clássico sistema **Ball and Beam (Bola e Barra)** utilizando **ESP32**, **Servo Motor** e **Sensor Sharp GP2Y0A41SK0F**.

O objetivo é manter uma bola posicionada em um ponto específico da barra através do controle automático da inclinação da estrutura.

## Demonstração

> Adicione aqui um GIF ou vídeo do sistema funcionando.

---

## Como funciona

O sistema realiza continuamente os seguintes passos:

1. Mede a posição da bola utilizando o sensor Sharp.
2. Filtra a leitura para reduzir ruídos.
3. Calcula o erro em relação ao setpoint.
4. Executa o controlador PID.
5. Ajusta a inclinação da barra por meio de um servomotor.
6. Repete o processo a cada 20 ms.

Fluxo simplificado:

```text
Sensor Sharp
      ↓
Filtro EMA
      ↓
Controlador PID
      ↓
Servo Motor
      ↓
Inclinação da Barra
      ↓
Movimento da Bola
```

---

## Hardware Utilizado

* ESP32
* Servo Motor SG90/MG995 (ou equivalente)
* Sensor Sharp GP2Y0A41SK0F
* Barra de alumínio ou MDF
* Bola de aço ou ping-pong
* Fonte de alimentação adequada para o servo

---

## Ligações

| Componente   | ESP32         |
| ------------ | ------------- |
| Sharp OUT    | GPIO 35       |
| Servo Signal | GPIO 14       |
| Sharp VCC    | 5V            |
| Sharp GND    | GND           |
| Servo VCC    | Fonte externa |
| Servo GND    | GND comum     |

### Atenção

Não é recomendado alimentar o servo diretamente pelo ESP32.

Utilize uma fonte externa e conecte os GNDs em comum.

---

## Bibliotecas Utilizadas

```cpp
#include <SharpIR.h>
#include <PID_v1.h>
#include <Servo.h>
```

Instale através do Library Manager da IDE Arduino ou via Platform IO Registry.

---

## Configuração do PID

Os parâmetros iniciais utilizados foram:

```cpp
double Kp = 3.0;
double Ki = 0.0;
double Kd = 1.0;
```

Esses valores provavelmente precisarão ser ajustados de acordo com:

* Comprimento da barra
* Peso da bola
* Tipo do servo
* Geometria do sistema

---

## Filtragem da Leitura

Foi utilizado um filtro exponencial simples:

```cpp
distanciaFiltrada =
  (1.0 - alpha) * distanciaFiltrada +
  alpha * leitura;
```

Valor utilizado:

```cpp
alpha = 0.2
```

Valores menores produzem leituras mais suaves, porém aumentam o atraso.

---

## Ajustando o PID

### Sistema muito lento

Aumente:

```cpp
Kp
```

### Sistema oscila muito

Reduza:

```cpp
Kp
```

ou aumente:

```cpp
Kd
```

### Sistema não consegue eliminar erro permanente

Adicione:

```cpp
Ki
```

com valores pequenos.

---

## Monitor Serial

O sistema imprime:

```text
Leitura: 14.7 cm
Filtrada: 14.9 cm
Setpoint: 15.0 cm
Erro: 0.1 cm
PID: 2.4
Servo: 92°
```

Essas informações são úteis para ajuste e depuração.

## Problemas Comuns

### Leitura muito instável

* Verifique a alimentação do sensor.
* Ajuste o valor de alpha.
* Certifique-se de que a bola está dentro da faixa de medição.

### Servo vibrando

* Utilize fonte externa.
* Verifique o aterramento comum.
* Ajuste os ganhos PID.

### Bola não estabiliza

* Recalibre o setpoint.
* Revise a geometria da barra.
* Refaça a sintonia do PID.

---

## Contribuições

Caso você implemente melhorias ou utilize outro sensor, fique à vontade para compartilhar os resultados.
