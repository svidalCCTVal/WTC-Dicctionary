#include <Arduino.h>
#include "Servo.h"

// Lista de comandos 
const char F_cmd[4]= "FFF";
const char R_cmd[4]= "RRR";
const char V_cmd[4]= "VVV";
const char T_cmd[4]= "TTT";
const char A_cmd[4]= "AAA";
const char I_cmd[4]= "III";
const char P_cmd[4]= "PPP"; // Parar
const char H_cmd[4]= "HHH"; // Help 

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

// Estados 
enum Estado{
  Reposo, 
  Derecha,
  DerechaA, 
  DerechaB,
  Izquierda, 
  IzquierdaA, 
  IzquierdaB, 
  Freno, 
  Error  
};

// Robot 
struct robot{
  Estado estado;
  int velocidad; 
  int tiempo_operacion; 
  int revoluciones[3];
  int frecuencias[3]; 

  /*VARIABLES GLOBALES DE LUCAS*/
  /*
  int speedsDerecha[10]={1500+160, 1500+120, 1500+220};
  int speedsIzquierda[10]={1500-160, 1500-120, 1500-220};
  int mode = 0;
  unsigned long previousMillisBrushless = 0;
  const unsigned long intervalBrushless = 500;
  */ 
};

// String utilities
int asignar_frecuencias(char * input_data, size_t len);
int asignar_revoluciones(char * input_data, size_t len);

// Instacia global de objeto Robot
robot Robot; 

// -------------------------  SETUP  ----------------------------
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

// -------------------------  LOOP  ----------------------------
void loop() {

  /* acá incluir polling de medición de frecuencia */

  // Parseo de datos recibidos por serial
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

    if (!strcmp(input_cmd,F_cmd)) // FFF asigna frecuencias pero no inicia operación
    {
      Serial.println("FFF yes");
      Serial.println(input_data);
      if(asignar_frecuencias(input_data, sizeof(input_data))){
        Robot.estado = Reposo; 
        Serial.println(A_cmd);
      } else
      {
        Serial.println(H_cmd);
      }
      
    }
    else if (!strcmp(input_cmd,R_cmd)) // RRR asigna cantidad revoluciones en operación, no inicia operación
    {
      Serial.println("RRR yes");
      if(asignar_revoluciones(input_data, sizeof(input_data))){       
        Robot.estado = Reposo; 
        Serial.println(A_cmd);
      }; 
    } 
    else if (!strcmp(input_cmd,V_cmd)) // VVV asigna velocidad, no inicia operación
    {
      Serial.println("VVV yes");

    } 
    else if (!strcmp(input_cmd,T_cmd)) // TTT asigna tiempo de la rutina de trabajo
    {
      Serial.println("TTT yes");
    } 
    else if (!strcmp(input_cmd,I_cmd)) // III inicia rutina de movimiento
    {
      Serial.println("III yes");
    }
    else if (!strcmp(input_cmd,P_cmd)) // PPP detiene rutina da lo mismo el momento donde llegue el mensaje
    {
      Serial.println("PPP yes");

    }
    else if (!strcmp(input_cmd,A_cmd)) // AAA es respuesta del 
    {
      Serial.println("AAA yes");
    }
  }

  switch (Robot.estado)
  {
  case Reposo:
    // apagar todos los motores
    break;
  
  case Derecha: 
    break;

  case DerechaA: 
    break;

  case DerechaB: 
    break;

  case Izquierda: 
    break;

  case IzquierdaA: 
    break;

  case IzquierdaB: 
    break;

  default:
    break;
  }
}


// Funciones de diccionario
int asignar_frecuencias(char * input_data, size_t len){ 

  return 1;
}

int asignar_revoluciones(char * input_data, size_t len){
  /* code */
  return 1; 
}

int asignar_velocidad(char * input_data, size_t len){
  /* code */
  return 1; 
}

int asignar_tiempo(char * input_data, size_t len){
  /* code */
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