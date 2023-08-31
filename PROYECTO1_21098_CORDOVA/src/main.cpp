//***************************************************************
//KAREN LEONOR CÓRDOVA LÓPEZ
// PROYECTO 1-D2-SENSOR DE TEMPERATURA
//****************************************************************
//Librerias
#include <Arduino.h>
#include "driver/ledc.h"
#include "esp_adc_cal.h"
#include "config.h"

//*****************************************************************
// Definición de etiquetas 

#define SNLM35 35
#define BTN_TEMP 13 
#define RED 25
#define GREEN 27
#define BLUE 15
#define Mservo 32 
//***********************************************************************************
//Definición de los pines para los segmentos y dígitos comunes del display

int segmentPins[7] = {18, 19, 21, 22, 23, 12, 26};
int commonPins[3] = {2, 4, 5};
int Dot = 14;
//*****************************************************************************************+
// Variables para la temperatura/variables globales
float TempC_LM35 = 0.0;
int temp = 0;
int placeValuesofTemp[4];
//*******************************************************************************************************
//Variables globales
int tempRefresh = 1000; // Actualizar la temperatura cada 1 seg
int sevSegRefresh = 5;

unsigned long time_now = 0;   //varible para almacenar el tiempo actual 
bool buttonPressed = false;    //bandera para indicar si el boton esta presionado 
bool temperatureTaken = false; //bandera para indicar si se ha tomado la temperatura
bool temperatureUpdated = false; // Bandera para mostrar la temperatura actualizada en el monitor serial
unsigned long lastButtonPressTime = 0;  //Tiempo en milisegundos
unsigned long debounceDelay = 50;  // Tiempo de espera para evitar rebotes del boton 
bool displaysOn = false;            //Bandera para indicar si los displays estan encendidos
int lastButtonState = HIGH;         //Estado del boton 


// set up the 'counter' feed
AdafruitIO_Feed *tempCanal = io.feed("Temperatura");
AdafruitIO_Feed *tempCanal2 = io.feed("angulo");


//*****************************************************************************************************************
// función para calibrar el valor ADC (convertir analogico a digital)
uint32_t readADC_Cal(int ADC_Raw) {
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

//***********************************************************************************************************************************
// Configuración 

void setup() {
  analogReadResolution(12);

  for (int i = 0; i < 7; i++) {
    pinMode(segmentPins[i], OUTPUT);
  }

  for (int i = 0; i < 3; i++) {
    pinMode(commonPins[i], OUTPUT);
  }

  pinMode(Dot, OUTPUT);
  pinMode(BTN_TEMP, INPUT_PULLUP);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  // Configuración de los temporizadores del LEDC (PWM)
  ledcSetup(LEDC_CHANNEL_0, 5000, 13);
  ledcSetup(LEDC_CHANNEL_1, 5000, 13);
  ledcSetup(LEDC_CHANNEL_2, 5000, 13);
  ledcAttachPin(RED, LEDC_CHANNEL_0);
  ledcAttachPin(GREEN, LEDC_CHANNEL_1);
  ledcAttachPin(BLUE, LEDC_CHANNEL_2);

  // Configuración del módulo LEDC para el servo
  ledcSetup(3, 50, 10); // Canal 3 para el servo, frecuencia 50 Hz, resolución de 10 bits
  ledcAttachPin(Mservo, 3); // Asignar el pin del servo al canal 3

  Serial.begin(115200);
   // wait for serial monitor to open
  Serial.print("Connecting to Adafruit IO");
  // connect to io.adafruit.com
  io.connect();
  
  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  // we are connected
  Serial.println();
  Serial.println(io.statusText());

}
//Loop Principal 
//*************************************************************************************************
void loop() {

  int reading = digitalRead(BTN_TEMP); //Leer el estado actual de boton 
  unsigned long currentTime = millis(); //obtener el tiempo actual en milisegundos

 

  if (!displaysOn && reading == LOW) {
    displaysOn = true;  // Cambiar el estado para encender los displays

  
 
  }
// Si los displays estan encendidos 
  if (displaysOn) {
    // Verifica si el estado actual del boton ha cambiado respecto al anterior
    if (reading != lastButtonState) {
      lastButtonState = reading;  //Actualizar el estado del botón, anterior
        // Si el bonton a sido presionado y ha pasado suficiente tiempo desde la ultima vez (no rebotes)
      if (reading == LOW && (currentTime - lastButtonPressTime) > debounceDelay) {
        lastButtonPressTime = currentTime;  //Actualizar el tiempo de la ultima actualización 

        // Actualizar la bandera para indicar que se debe tomar la temperatura nuevamente
        temperatureTaken = false;
        //Actualizar la bandera para indicar que la temperatura no se ha actualizado en el monitor serial  
        temperatureUpdated = false;
      }
    }
  }

  if (temperatureTaken) {
 
    // Mostrar la temperatura si ha sido tomada y el botón se ha presionado
    int number[10][7] = {
      {1, 1, 1, 1, 1, 1, 0}, // 0
      {0, 0, 0, 0, 1, 1, 0}, // 1
      {1, 1, 0, 1, 1, 0, 1}, // 2
      {1, 1, 1, 1, 0, 0, 1}, // 3
      {0, 1, 1, 0, 0, 1, 1}, // 4
      {1, 0, 1, 1, 0, 1, 1}, // 5
      {1, 0, 1, 1, 1, 1, 1}, // 6
      {1, 1, 1, 0, 0, 0, 0}, // 7
      {1, 1, 1, 1, 1, 1, 1}, // 8
      {1, 1, 1, 0, 0, 1, 1}  // 9
    };

    int commonPinStates[3][3] = {
      {1, 0, 0}, // habilitar el dígito 1
      {0, 1, 0}, // habilitar el dígito 2
      {0, 0, 1}  // habilitar el dígito 3
    };

    for (int n = 0; n < 3; n++) {  // Loop iterar a traves de los tres dígitos 
      for (int i = 0; i < 3; i++) {   //Loop para iterar a traves de los pines comunes (caton comun)
        digitalWrite(commonPins[i], commonPinStates[n][i]);  //Establecer el estado de los pines comunes segun el digito actual 
        (n == 1) ? digitalWrite(Dot, 1) : digitalWrite(Dot, 0);  //Encender o apagar el punto decimal en el segundo digito
      }

      for (int j = 0; j < 7; j++) {  //Loop para iterar a traves de los segmentos de cada display
        digitalWrite(segmentPins[j], number[placeValuesofTemp[n]][j]);  //Encender o apagar los segmnetos segun el digito y el numero 
      }

      delay(sevSegRefresh);  //Mantener los segmnetos encendidos durante un breve periodo  

      for (int i = 0; i < 7; i++) {
        digitalWrite(segmentPins[i], LOW); // Apagar todos los segmentos antes de cambiar de visualización
      }
    }

    // Control de color del LED RGB y movimiento gradual del servo según la temperatura
    int servoAngle;

    if (TempC_LM35 < 23) {
      servoAngle = map(int(TempC_LM35), 0, 22, 0, 10);
    } else if (TempC_LM35 <= 23) {
      servoAngle = map(int(TempC_LM35), 22, 23, 10, 20);
    } else if (TempC_LM35 <= 24) {
      servoAngle = map(int(TempC_LM35), 23, 24, 20, 30);
    } else if (TempC_LM35 <= 25) {
      servoAngle = map(int(TempC_LM35), 24, 25, 30, 40);
    } else if (TempC_LM35 <= 26) {
      servoAngle = map(int(TempC_LM35), 25, 26, 40, 50);
    } else if (TempC_LM35 <= 27) {
      servoAngle = map(int(TempC_LM35), 26, 27, 50, 60);
    } else if (TempC_LM35 <= 28) {
      servoAngle = map(int(TempC_LM35), 27, 28, 60, 70);
    } else if (TempC_LM35 <= 29) {
      servoAngle = map(int(TempC_LM35), 28, 29, 70, 80);
    } else if (TempC_LM35 <= 30) {
      servoAngle = map(int(TempC_LM35), 29, 30, 80, 90);
    } else if (TempC_LM35 <= 31) {
      servoAngle = map(int(TempC_LM35), 30, 31, 90, 100);
    } else if (TempC_LM35 <= 32) {
      servoAngle = map(int(TempC_LM35), 31, 32, 100, 110);
    } else if (TempC_LM35 <= 33) {
      servoAngle = map(int(TempC_LM35), 32, 33, 110, 120);
    } else if (TempC_LM35 <= 34) {
      servoAngle = map(int(TempC_LM35), 33, 34, 120, 130);
    } else if (TempC_LM35 <= 35) {
      servoAngle = map(int(TempC_LM35), 34, 35, 130, 140);
    } else if (TempC_LM35 <= 36) {
      servoAngle = map(int(TempC_LM35), 35, 36, 140, 150);
    } else if (TempC_LM35 <= 37) {
      servoAngle = map(int(TempC_LM35), 36, 37, 150, 160);
    } else if (TempC_LM35 <= 38) {
      servoAngle = map(int(TempC_LM35), 37, 38, 160, 170);
    } else if (TempC_LM35 <= 39) {
      servoAngle = map(int(TempC_LM35), 38, 39, 170, 180);
    } else {
      servoAngle = 180;
    }

    servoAngle = constrain(servoAngle, 0, 180); // Limitar el ángulo del servo entre 0 y 180 grados
    ledcWrite(3, servoAngle); // Mover el servo gradualmente hacia el ángulo calculado
     
     //Verificar si la información de la temperatura ya ha sido actualizada
    if (!temperatureUpdated) {   
      Serial.print("Temperatura: ");   
      Serial.print(TempC_LM35);    //Muestra el valor de temperatura en celsius 
      Serial.print(" °C, Ángulo: ");
      Serial.print(servoAngle);     //Muestra el valorr del angulo 
      Serial.println("°");
      temperatureUpdated = true;   //Marcar que la información ha sido actualizada 
      
           // it should always be present at the top of your loop
          // function. it keeps the client connected to
          // io.adafruit.com, and processes any incoming data.
          io.run();

          // save count to the 'counter' feed on Adafruit IO
          Serial.print("sending -> ");
          Serial.println(TempC_LM35);
          Serial.println(servoAngle);
          tempCanal ->save(TempC_LM35);
          tempCanal2->save(servoAngle);

          delay(3000);

    }

    if (TempC_LM35 < 23.0) {
      ledcWrite(LEDC_CHANNEL_0, 0);     // Apagar LED rojo
      ledcWrite(LEDC_CHANNEL_1, 255);   // Encender LED verde
      ledcWrite(LEDC_CHANNEL_2, 0);     // Apagar LED azul
    } else if (TempC_LM35 >= 23.1 && TempC_LM35 <= 26) {
      ledcWrite(LEDC_CHANNEL_0, 255);   // Encender LED rojo
      ledcWrite(LEDC_CHANNEL_1, 0);   // Encender LED verde
      ledcWrite(LEDC_CHANNEL_2, 255);     // Apagar LED azul
    } else if(TempC_LM35 >= 26) {
      ledcWrite(LEDC_CHANNEL_0, 255);   // Encender LED rojo
      ledcWrite(LEDC_CHANNEL_1, 0);     // Apagar LED verde
      ledcWrite(LEDC_CHANNEL_2, 0);     // Apagar LED azul
    }

  } else {
    // Apagar todos los segmentos si la temperatura no se ha tomado
    // o si los displays no están encendidos
    for (int i = 0; i < 7; i++) {
      digitalWrite(segmentPins[i], LOW);
    }
    digitalWrite(Dot, LOW);

    // Apagar el LED RGB y detener el servo si la temperatura no se ha tomado
    ledcWrite(LEDC_CHANNEL_0, 0);
    ledcWrite(LEDC_CHANNEL_1, 0);
    ledcWrite(LEDC_CHANNEL_2, 0);
    ledcWrite(3, 0);
  }

 
 

  // Leer y mostrar la temperatura si el botón se ha presionado
  if (displaysOn && !temperatureTaken && reading == LOW) {
    int SNLM35_Raw = analogRead(SNLM35);  //Calcular el voltaje correpsondiente al valor del sensor (real)
    float Voltage = readADC_Cal(SNLM35_Raw); // Calcular la temperatura en grados Celsius usando la relación del LM35
    TempC_LM35 = ((SNLM35_Raw*5000)/4095)/10;
    
    // Calcular el valor de la temperatura multiplicando por 100 para trabajar con decimales  
    temp = TempC_LM35 * 100;
     //Descomponer el valor de la temperatura en sus digitos individuales 
    placeValuesofTemp[3] = ((temp) / 1) % 10;   //dígito de la unidades (0-9)
    placeValuesofTemp[2] = ((temp) / 10) % 10;  // digito de las decenas (0-9)
    placeValuesofTemp[1] = ((temp) / 100) % 10;  //digito de las centenas (0-9)
    placeValuesofTemp[0] = ((temp) / 1000) % 10;  //digito de los millares (0-9)
     //Marca que la temperatura ha sido tomada 
    temperatureTaken = true;
  }

 
}
  