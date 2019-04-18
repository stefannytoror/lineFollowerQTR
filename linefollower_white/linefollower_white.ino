#include <QTRSensors.h>

#define NUM_SENSORS   5     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin
#define QTR_EMITTERS_ON 1

#define velocidadBase 40
#define velMaxDerecha 80
#define velMaxIzquierda 80

//Constantes de error y de derivada del error
#define kp 0.025
#define kd 0.95

int interseccion = 0;
int botella = 0;
int finally = 0;
// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {A1,A2,A3,A4,A5},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];
int ultimoError = 0;

void setup() {
  // calibra 10 sg
  for (int i = 0; i < 400; i++) {  // make the calibration take about 10 seconds
    qtrrc.calibrate();             // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);
  delay(2000);
}

void loop() {
  // si encuentra botella en el camino la saca
  if(ultrasoundRead(J1) <= 7) {
    botella++;
    sacarBotella();
    }

  // Lee los sensores
  unsigned int sensors[NUM_SENSORS];

  // Ubica la posicion donde está la linea negra
  int position = qtrrc.readLine(sensors,QTR_EMITTERS_ON,1);

  if (sensors[0] < 200 && sensors[1] < 200 &&sensors[2] < 200 && sensors[3] < 200){
      interseccion++;
      motorSpeed(M1 , 40);
      motorSpeed(M2 , 40);
      goForward(M1 , M2);
      delay(200);

      if(interseccion == 1) {
        motorSpeed(M1 , 40);
        motorSpeed(M2 , 40);
        turnLeft(M2 , M1);
        delay(800);
        }

        if(botella == 3) {
          interseccion == 4;
        }
        else if(interseccion == 5) {
          finally == 1;
        motorSpeed(M1 , 40);
        motorSpeed(M2 , 40);
        turnRight(M2 , M1);
        delay(1000);
        }
    }

  int error = position - 2000; // con 5 sensores 2000 con 6 2500 y con los 8 3500
  int velMotor = kp*error + kd*(error-ultimoError);
  ultimoError = error;

  // Asigna a cada uno la velocidad dependiendo de
  // donde se encuentre la linea negra
  int motorDerecho = velocidadBase + velMotor;
  int motorIzquierdo = velocidadBase - velMotor;

  if(motorDerecho > velMaxDerecha) motorDerecho = velMaxDerecha;
  if(motorIzquierdo > velMaxIzquierda) motorIzquierdo = velMaxIzquierda;
  if(motorDerecho < 0) motorDerecho = 0;
  if(motorIzquierdo < 0) motorIzquierdo = 0;

  // Se le pasa la velocidad calculada, a los motores
  motorSpeed(M1 ,motorDerecho);
  motorSpeed(M2 ,motorIzquierdo);
  motorOn(M1,FORWARD);
  motorOn(M2,FORWARD);
}

void sacarBotella() {
  if(interseccion >= 5) {

    motorSpeed(M3 , 60);
    motorSpeed(M1 , 60);
    motorSpeed(M2 , 40);

     motorsOff(M1 , M2);
     delay(100);
     motorOn(M3 , REVERSE);
     delay(400);
     goReverse(M1 , M2);
     delay(1000);
     motorsOff(M1 , M2);
     motorOn(M3 , FORWARD);
     delay(400);

    } else {
        motorSpeed(M3,60);
        motorSpeed(M1,60);
        motorSpeed(M2,40);
        motorsOff(M1,M2);
        delay(100);
        motorOn(M3,REVERSE);
        delay(400);
        goForward(M1,M2);
        delay(1000);
        motorsOff(M1,M2);
        motorOn(M3,FORWARD);
        delay(400);
        goReverse(M1,M2);
        delay(1000);
    }
  }
