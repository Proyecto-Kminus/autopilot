#include <SoftwareSerial.h>
SoftwareSerial bluetooth(2, 3);

//LINK DE ENLACE A LA WEB
https://raulperezcortes.github.io/Autopilot/

// ═══════════════════════════════════════════════════════
//   PARÁMETROS DE COMPORTAMIENTO
// ═══════════════════════════════════════════════════════

// Velocidad máxima (0-255)
const int   VELOCIDAD_MAX = 255;         // ❌ Al tope: el robot va demasiado rápido y es difícil de controlar

// Velocidad mínima (0-255)
const int   VELOCIDAD_MINIMA = 200;      // ❌ Demasiado alta: el robot pega un tirón enorme con cualquier toque del joystick

// Cuánto influye el eje X (giro) respecto al Y (avance) (0.0-1.0)
// 1.0 = giro a tope, 0.5 = giro más suave
const float FACTOR_GIRO = 1.0;          // ❌ Giro a tope: al mover el joystick en diagonal un motor se para completamente

// Zona muerta general del joystick: valores por debajo se ignoran (0.0-1.0)
// Evita que el robot se mueva por pequeñas derivas del joystick en reposo
const float ZONA_MUERTA = 0.1;

// Zona muerta del eje X: si X está por debajo de este valor el robot va
// recto sin girar aunque el joystick no esté perfectamente centrado (0.0-1.0)
const float ZONA_MUERTA_X = 0.1;

// Zona muerta del eje Y: si Y está por debajo de este valor el robot no
// avanza ni retrocede aunque el joystick no esté perfectamente centrado (0.0-1.0)
const float ZONA_MUERTA_Y = 0.1;

// Intervalo de envío de distancia al móvil (ms)
// Más bajo = más reactivo ante obstáculos, más tráfico Bluetooth
const unsigned long INTERVALO_ENVIO = 2000;   // ❌ Demasiado alto: el sensor tarda 2 segundos en detectar el obstáculo

// Intervalo entre disparos del sensor de ultrasonidos (ms)
const unsigned long INTERVALO_TRIGGER = 2000; // ❌ Demasiado alto: el sensor solo mide una vez cada 2 segundos

// Distancia que se reporta cuando el sensor no detecta nada (cm)
const long DISTANCIA_SIN_ECO = 400;

// Timeout del pulso del ultrasonidos (microsegundos)
// 20000 us = ~340 cm máximo medible
const long TIMEOUT_ULTRASONIDO = 20000;

// ═══════════════════════════════════════════════════════
//   PINES — cambia si tu cableado es diferente
// ═══════════════════════════════════════════════════════

const int pinTrigger = 11;
const int pinEcho    = 12;
const int motorIzq1  = 5;
const int motorIzq2  = 6;
const int motorDer1  = 9;
const int motorDer2  = 10;

// ═══════════════════════════════════════════════════════
//   ESTADO INTERNO
// ═══════════════════════════════════════════════════════

float btX = 0.0;
float btY = 0.0;
unsigned long ultimoTrigger = 0;
unsigned long ultimoEnvio   = 0;
long distancia = DISTANCIA_SIN_ECO;

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  bluetooth.setTimeout(5);

  pinMode(motorIzq1, OUTPUT);
  pinMode(motorIzq2, OUTPUT);
  pinMode(motorDer1, OUTPUT);
  pinMode(motorDer2, OUTPUT);
  pinMode(pinTrigger, OUTPUT);
  pinMode(pinEcho, INPUT);
  digitalWrite(pinTrigger, LOW);
}

void loop() {
  unsigned long ahora = millis();

  // 1. DISPARAR SENSOR
  if (ahora - ultimoTrigger >= INTERVALO_TRIGGER) {
    ultimoTrigger = ahora;
    digitalWrite(pinTrigger, LOW);
    delayMicroseconds(2);
    digitalWrite(pinTrigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrigger, LOW);

    long tiempo = pulseIn(pinEcho, HIGH, TIMEOUT_ULTRASONIDO);
    distancia = (tiempo == 0) ? DISTANCIA_SIN_ECO : tiempo / 59;
  }

  // 2. ENVIAR DISTANCIA
  if (ahora - ultimoEnvio >= INTERVALO_ENVIO) {
    ultimoEnvio = ahora;
    bluetooth.print("D:");
    bluetooth.println(distancia);
  }

  // 3. LEER JOYSTICK
  if (bluetooth.available() > 0) {
    String cadena = bluetooth.readStringUntil('\n');
    int coma = cadena.indexOf(',');
    if (coma != -1) {
      btX = constrain(cadena.substring(0, coma).toFloat(), -1.0, 1.0); // ❌El sentido del eje X estaá invertido
      btY = constrain(cadena.substring(coma + 1).toFloat(), -1.0, 1.0);
    }
  }

  // 4. APLICAR ZONAS MUERTAS
  float ejeX = (abs(btX) < ZONA_MUERTA || abs(btX) < ZONA_MUERTA_X) ? 0.0 : btX;
  float ejeY = -((abs(btY) < ZONA_MUERTA || abs(btY) < ZONA_MUERTA_Y) ? 0.0 : btY);

  // 5. CALCULAR VELOCIDADES
  int velocidadIzq = (ejeY + ejeX * FACTOR_GIRO) * VELOCIDAD_MAX;
  int velocidadDer = (ejeY - ejeX * FACTOR_GIRO) * VELOCIDAD_MAX;

  velocidadIzq = constrain(velocidadIzq, -VELOCIDAD_MAX, VELOCIDAD_MAX);
  velocidadDer = constrain(velocidadDer, -VELOCIDAD_MAX, VELOCIDAD_MAX);

  // 6. APLICAR VELOCIDAD MÍNIMA (solo si hay intención de moverse)
  if (velocidadIzq != 0 && abs(velocidadIzq) < VELOCIDAD_MINIMA)
    velocidadIzq = (velocidadIzq > 0) ? VELOCIDAD_MINIMA : -VELOCIDAD_MINIMA;
  if (velocidadDer != 0 && abs(velocidadDer) < VELOCIDAD_MINIMA)
    velocidadDer = (velocidadDer > 0) ? VELOCIDAD_MINIMA : -VELOCIDAD_MINIMA;

  actualizarMotorPWM(motorIzq1, motorIzq2, velocidadIzq);
  actualizarMotorPWM(motorDer1, motorDer2, velocidadDer);
}

void actualizarMotorPWM(int pinA, int pinB, int velocidad) {
  if (velocidad > 0) {
    analogWrite(pinA, velocidad);
    digitalWrite(pinB, LOW);
  } else if (velocidad < 0) {
    digitalWrite(pinA, LOW);
    analogWrite(pinB, abs(velocidad));
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
}
