#include <Arduino.h>
#include "Servo.h"

// Lista de comandos 
const char F_cmd[4]= "FFF";
const char R_cmd[4]= "RRR";
const char V_cmd[4]= "VVV";
const char T_cmd[4]= "TTT";
const char A_cmd[4]= "AAA";

// Pines para control Puente-H de motores de desplazamiento
const int ENA = 3;   // Enable motor izquierdo
const int IN1 = 4;   // Dirección motor izquierdo (hacia adelante)
const int IN2 = 2;   // Dirección motor izquierdo (hacia atrás)
const int IN3 = 13;  // Dirección motor derecho (hacia adelante)
const int IN4 = A4;  // Dirección motor derecho (hacia atrás)
const int ENB = 5;   // Enable motor derecho

// Pines para control PWM de motores brushless
const uint8_t VescOutputPinA = 11; // Motor brushless de abajo
const uint8_t VescOutputPinB = 6;  // Motor brushless de arriba

// Pines para interruptores
const int SWCHA1 = 8;
const int SWCHA2 = 9;
const int SWCHB1 = A3;
const int SWCHB2 = A2;

// Velocidad del motor (PWM)
int velocidadMotorA = 150;
int velocidadMotorB = 150;
const int velocidadReducida = 135;

// Brushless
int NroModos = 3;
int Modo = 50;
int speedsDerecha[10] = {1500+Modo, 1500+Modo, 1500+Modo}; 
int speedsIzquierda[10] = {1500-Modo, 1500-Modo, 1500-Modo};
int mode = 0;
unsigned long previousMillisBrushless = 0;
const unsigned long intervalBrushless = 500;

// Estado global
bool desplazamientoActivo = false;
bool brushlessActivo = false;
bool estadoMotor = true;

// Objetos Servo para los ESC
Servo escA;
Servo escB;

// Funciones

void moverMotorA(bool haciaDerecha, int velocidad);
void moverMotorB(bool haciaDerecha, int velocidad);
void detenerMotorA();
void detenerMotorB();
void detenerMotores();
void controlarBrushless(bool haciaDerecha);
void detenerBrushless();
//void controlarDesplazamiento();


// String utilities
int asignar_frecuencias(char * input_data, size_t len);


void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  escA.attach(VescOutputPinA);
  escB.attach(VescOutputPinB);
  escA.writeMicroseconds(1500);
  escB.writeMicroseconds(1500);

  pinMode(SWCHA1, INPUT_PULLUP);
  pinMode(SWCHA2, INPUT_PULLUP);
  pinMode(SWCHB1, INPUT_PULLUP);
  pinMode(SWCHB2, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println("AAA");
}

void loop() {
  // reply only when you receive data:
  if (Serial.available() > 0) {
    char input_serial[64];
    size_t len = Serial.readBytesUntil('\n', input_serial, sizeof(input_serial));
    input_serial[len]='\0';

    char input_cmd[4];
    strncpy(input_cmd,input_serial,3);
    input_cmd[3]='\0';

    char input_data[32];
    size_t data_len = len-3;

    strncpy(input_data,input_serial+3, data_len);
    input_data[data_len] = '\0';

    if (!strcmp(input_cmd,F_cmd))
    {
      Serial.println("FFF yes");
      Serial.println(input_data);
      //if(asignar_frecuencias(input_data, sizeof(input_data))){
      //  Serial.println(A_cmd);
      //};
    } //RRR
    else if (!strcmp(input_cmd,R_cmd))
    {
      Serial.println("FFF yes");
    } /* //VVV
    else if (!strcmp(input_cmd,F_cmd))
    {
      Serial.println("FFF yes");
    } //TTT
    else if (!strcmp(input_cmd,F_cmd))
    {
      Serial.println("FFF yes");
    } //III
    else if (!strcmp(input_cmd,F_cmd))
    {
      Serial.println("FFF yes");
    }//PPP
    else if (!strcmp(input_cmd,F_cmd))
    {
      Serial.println("FFF yes");
    }//AAA
    else if (!strcmp(input_cmd,F_cmd))
    {
      Serial.println("FFF yes");
    }
    */
  }
}


// Funciones de diccionario
int asignar_frecuencias(char * input_data, size_t len){ 
    Serial.print("Datos recibidos: ");
    Serial.println(input_data);
    Serial.print("Largo máximo de buffer: ");
    Serial.println(len);
    return 1;
}

// Funciones de control de movimiento


void moverMotorA(bool haciaDerecha, int velocidad) {
  analogWrite(ENA, velocidad);
  digitalWrite(IN1, haciaDerecha ? HIGH : LOW);
  digitalWrite(IN2, haciaDerecha ? LOW : HIGH);
}

void moverMotorB(bool haciaDerecha, int velocidad) {
  analogWrite(ENB, velocidad);
  digitalWrite(IN3, haciaDerecha ? HIGH : LOW);
  digitalWrite(IN4, haciaDerecha ? LOW : HIGH);
}

void detenerMotorA() {
  analogWrite(ENA, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
}

void detenerMotorB() {
  analogWrite(ENB, 255);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}

void controlarBrushless(bool haciaDerecha) {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillisBrushless >= intervalBrushless) {
    previousMillisBrushless = currentMillis;
    mode = (mode + 1) % NroModos;
    escA.writeMicroseconds(haciaDerecha ? speedsDerecha[mode] : speedsIzquierda[mode]);
    escB.writeMicroseconds(haciaDerecha ? speedsDerecha[mode] : speedsIzquierda[mode]);
  }
}
 
void detenerBrushless() {
  escA.writeMicroseconds(1500);
  escB.writeMicroseconds(1500);
}

void detenerMotores() {
  analogWrite(ENB, 0);  // desactiva la energía 
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);  // desactiva la energía 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}