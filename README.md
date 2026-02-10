# Robot Péndulo Invertido (Self-Balancing Robot) con Control PID

![Arduino](https://img.shields.io/badge/Arduino-Nano-blue) ![License](https://img.shields.io/badge/License-MIT-green) ![Status](https://img.shields.io/badge/Status-Completed-success)

Este repositorio contiene el código fuente y la documentación de diseño para un **Robot de Péndulo Invertido**. El proyecto implementa un sistema de control de lazo cerrado (PID) para estabilizar una estructura verticalmente inestable sobre dos ruedas.

## 📋 Descripción del Proyecto

El objetivo principal de este proyecto fue diseñar e implementar un sistema mecatrónico capaz de mantener el equilibrio vertical de forma autónoma. Se utiliza un **Arduino Nano** como controlador central y un **IMU MPU6050** para la retroalimentación del ángulo de inclinación.

El sistema integra teoría de control clásico (PID) con electrónica de potencia y programación en C++ para microcontroladores.

### 🎯 Objetivos Logrados
* **Estabilidad:** Mantener el péndulo en posición vertical (SetPoint ≈ 180°).
* **Regulación:** Compensación de perturbaciones externas mediante ajuste dinámico de la velocidad de los motores.
* **Sintonización:** Ajuste experimental de las constantes $K_p, K_i, K_d$ para reducir oscilaciones y mejorar el tiempo de respuesta.

## 🛠️ Hardware Utilizado

| Componente | Descripción | Función |
|------------|-------------|---------|
| **Microcontrolador** | Arduino Nano (ATmega328) | Procesamiento de señal y cálculo del PID. |
| **Sensor IMU** | MPU6050 | Acelerómetro y Giroscopio (3 ejes) vía I2C. |
| **Driver de Motores** | Puente H L298N | Control de dirección y potencia (PWM) de los motores. |
| **Actuadores** | 2x Motorreductores DC | Relación 120:1 con llantas de goma. |
| **Alimentación** | Batería LiPo 3S (11.1V) | Alimentación de potencia para motores. |
| **Alimentación Lógica** | Batería 9V | Alimentación aislada para el Arduino. |

## ⚙️ Diagramas del Sistema

### Diagrama de Control (Lazo Cerrado)
El sistema opera leyendo la inclinación actual ($\theta_m$), comparándola con el punto de equilibrio deseado ($\theta_d$), y generando una señal de error ($e$) que alimenta al controlador PID.



### Conexiones Electrónicas
El esquema de conexión entre el MPU6050, el Puente H y el Arduino Nano.



## 💻 Implementación del Software

El código se basa en la lectura del **DMP (Digital Motion Processor)** del MPU6050 para obtener un ángulo limpio y sin ruido.

### Constantes PID
Los valores finales para la estabilización fueron obtenidos mediante prueba y error:

```cpp
double Kp = 60;   // Ganancia Proporcional: Reacción rápida al error
double Kd = 3.5;  // Ganancia Derivativa: Amortiguamiento de oscilaciones
double Ki = 250;  // Ganancia Integral: Corrección del error estacionario
