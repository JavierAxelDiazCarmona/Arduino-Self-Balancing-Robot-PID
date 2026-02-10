// =============================================================
// PROYECTO: Robot Péndulo Invertido
// AUTOR: Equipo 5incontroles (Reconstruido)
// =============================================================

#include <PID_v1.h>
#include <LMotorController.h> // Asegúrate de tener esta librería o el archivo .h/.cpp en la carpeta
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>

// Configuración I2C
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define MIN_ABS_SPEED 30

MPU6050 mpu;

// =============================================================
// AJUSTES Y CONSTANTES
// =============================================================
double MotorVelocidadIzq = 0.5; // Velocidad del motor izquierdo (factor)
double MotorVelocidadDer = 0.5; // Velocidad del motor derecho (factor)

// Ángulo de equilibrio en grados (valor ideal para mantener el equilibrio)
// NOTA: Este valor se debe ajustar experimentalmente según el centro de masa de tu robot.
double PuntoEquilibrio = 180.8; 

// Configuración del puerto serial para comunicación Bluetooth (pines 4 y 3)
SoftwareSerial Serial2(4, 3); 

// =============================================================
// CONTROL DE MOTORES (Pines)
// =============================================================
int ENA = 5;  // Pin para habilitar el motor izquierdo
int IN1 = 6;  // Pin de dirección del motor izquierdo
int IN2 = 7;  // Pin de dirección del motor izquierdo
int IN3 = 9;  // Pin de dirección del motor derecho
int IN4 = 8;  // Pin de dirección del motor derecho
int ENB = 10; // Pin para habilitar el motor derecho

// =============================================================
// CONSTANTES PID
// =============================================================
// Los valores de PID cambian con cada diseño físico (peso, altura, motores)
double Kp = 60;   // Coeficiente proporcional
double Kd = 3.5;  // Coeficiente derivativo
double Ki = 250;  // Coeficiente integral

// =============================================================
// VARIABLES GLOBALES
// =============================================================
int estado = 'g'; // Inicializa el estado del robot como detenido

// MPU control/status vars
bool dmpReady = false;  // Indica si la inicialización del DMP fue exitosa
uint8_t mpuIntStatus;   // Almacena el estado de la interrupción del MPU
uint8_t devStatus;      // Estado después de cada operación (0 = éxito, !0 = error)
uint16_t packetSize;    // Tamaño esperado del paquete DMP (por defecto 42 bytes)
uint16_t fifoCount;     // Cuenta los bytes actualmente en el FIFO
uint8_t fifoBuffer[64]; // Buffer de almacenamiento para los datos del FIFO

// Variables de orientación y movimiento
Quaternion q;           // Contenedor de quaternion [w, x, y, z]
VectorFloat gravity;    // Vector de gravedad [x, y, z]
float ypr[3];           // Contenedor para [yaw, pitch, roll]

// Variables PID
double originalSetpoint = PuntoEquilibrio; // Ángulo de equilibrio original
double setpoint = originalSetpoint;        // Ángulo de referencia actual
double movingAngleOffset = 0.1;            // Desviación del ángulo en movimiento
double input, output;                      // Entrada y salida del PID

// Configuración del objeto PID
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Factores de velocidad
double motorSpeedFactorLeft = MotorVelocidadIzq;
double motorSpeedFactorRight = MotorVelocidadDer;

// Controlador de los motores
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

// =============================================================
// INTERRUPCIONES
// =============================================================
volatile bool mpuInterrupt = false; // Indica si el pin de interrupción del MPU se ha activado

void dmpDataReady() {
    mpuInterrupt = true;
}

// =============================================================
// SETUP
// =============================================================
void setup() {
    Serial2.begin(9600); // Inicia comunicación Bluetooth
    // Serial.begin(9600); // Descomentar para depuración por USB

    // Inicia el bus I2C
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // Configura reloj I2C a 400kHz
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // Inicializa el sensor MPU6050
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // Configura los offsets del giroscopio (CALIBRACIÓN ESPECÍFICA DE TU SENSOR)
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    // Verifica si la inicialización fue exitosa
    if (devStatus == 0) {
        // Activa el DMP
        mpu.setDMPEnabled(true);

        // Habilita la detección de interrupciones en Arduino (Pin 2 digital es Int 0)
        attachInterrupt(0, dmpDataReady, RISING);
        
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();

        // Configura el PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10); // Tiempo de muestreo en ms
        pid.SetOutputLimits(-255, 255); // Límites PWM
    } else {
        // ERROR: 1 = carga inicial de memoria fallida, 2 = configuración de actualizaciones DMP fallida
        Serial.print(F("Error en la inicializacion del DMP (codigo "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

// =============================================================
// LOOP PRINCIPAL
// =============================================================
void loop() {
    // Si falló la inicialización, no hacer nada
    if (!dmpReady) return;

    // Espera a que haya datos del MPU.
    // Mientras NO haya interrupción, calculamos el PID y movemos motores
    // Esto asegura que los motores sigan respondiendo aunque el sensor sea lento.
    while (!mpuInterrupt && fifoCount < packetSize) {
        pid.Compute();
        motorController.move(output, MIN_ABS_SPEED);
    }

    // Resetear bandera de interrupción y leer estado
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    // Revisar desbordamiento (overflow)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("Desbordamiento de FIFO"));
    }
    // Si hay datos listos del DMP
    else if (mpuIntStatus & 0x02) {
        // Esperar a tener el paquete completo
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // Leer datos del FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        // Calcular ángulos reales usando el DMP
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Calcular el ángulo de inclinación (Input para el PID)
        // Se multiplica por 180/PI para convertir radianes a grados
        input = ypr[1] * 180/M_PI + 180;
    }
}