//KAREN LEONOR CÓRDOVA LÓPEZ
#include <Arduino.h>
#include "driver/ledc.h"
#include "esp_adc_cal.h"

#define SNLM35 35
#define BTN_TEMP 13 
#define RED 25
#define GREEN 27
#define BLUE 15
#define servoPin 32 

int segmentPins[7] = {18, 19, 21, 22, 23, 12, 26};
int commonPins[3] = {2, 4, 5};
int Dot = 14;

float TempC_LM35 = 0.0;
int temp = 0;
int placeValuesofTemp[4];

int tempRefresh = 1000; // Actualizar la temperatura cada 1 seg
int sevSegRefresh = 5;

unsigned long time_now = 0;
bool buttonPressed = false;
bool temperatureTaken = false;
bool temperatureUpdated = false; // Bandera para mostrar la temperatura actualizada en el monitor serial
unsigned long lastButtonPressTime = 0;
unsigned long debounceDelay = 50;
bool displaysOn = false;
int lastButtonState = HIGH;

uint32_t readADC_Cal(int ADC_Raw) {
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

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

  // Configuración de los temporizadores del LEDC
  ledcSetup(LEDC_CHANNEL_0, 5000, 13);
  ledcSetup(LEDC_CHANNEL_1, 5000, 13);
  ledcSetup(LEDC_CHANNEL_2, 5000, 13);
  ledcAttachPin(RED, LEDC_CHANNEL_0);
  ledcAttachPin(GREEN, LEDC_CHANNEL_1);
  ledcAttachPin(BLUE, LEDC_CHANNEL_2);

  // Configuración del módulo LEDC para el servo
  ledcSetup(3, 50, 10); // Canal 3 para el servo, frecuencia 50 Hz, resolución de 10 bits
  ledcAttachPin(servoPin, 3); // Asignar el pin del servo al canal 3

  Serial.begin(115200);
}

void loop() {
  int reading = digitalRead(BTN_TEMP);
  unsigned long currentTime = millis();

  if (!displaysOn && reading == LOW) {
    displaysOn = true;
  }

  if (displaysOn) {
    if (reading != lastButtonState) {
      lastButtonState = reading;

      if (reading == LOW && (currentTime - lastButtonPressTime) > debounceDelay) {
        lastButtonPressTime = currentTime;

        // Actualizar la temperatura al presionar el botón nuevamente
        temperatureTaken = false;
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

    for (int n = 0; n < 3; n++) {
      for (int i = 0; i < 3; i++) {
        digitalWrite(commonPins[i], commonPinStates[n][i]);
        (n == 1) ? digitalWrite(Dot, 1) : digitalWrite(Dot, 0);
      }

      for (int j = 0; j < 7; j++) {
        digitalWrite(segmentPins[j], number[placeValuesofTemp[n]][j]);
      }

      delay(sevSegRefresh);

      for (int i = 0; i < 7; i++) {
        digitalWrite(segmentPins[i], LOW); // Apagar todos los segmentos antes de cambiar de visualización
      }
    }

    // Control de color del LED RGB y movimiento gradual del servo según la temperatura
    int servoAngle;

    if (TempC_LM35 < 23) {
      servoAngle = map(int(TempC_LM35), 0, 22, 0, 0);
    }else if (TempC_LM35 <= 23) {
      servoAngle = map(int(TempC_LM35), 22, 23, 0, 3);
    } else if (TempC_LM35 <= 24) {
      servoAngle = map(int(TempC_LM35), 23, 24, 3, 10);
    } else if (TempC_LM35 <= 25) {
      servoAngle = map(int(TempC_LM35), 24, 25, 10, 20);
    } else if (TempC_LM35 <= 26) {
      servoAngle = map(int(TempC_LM35), 25, 26, 20, 30);
    } else if (TempC_LM35 <= 27) {
      servoAngle = map(int(TempC_LM35), 26, 27, 30, 40);
    } else if (TempC_LM35 <= 28) {
      servoAngle = map(int(TempC_LM35), 27, 28, 40, 50);
    } else if (TempC_LM35 <= 29) {
      servoAngle = map(int(TempC_LM35), 28, 29, 50, 60);
    } else if (TempC_LM35 <= 30) {
      servoAngle = map(int(TempC_LM35), 29, 30, 60, 70);
    } else if (TempC_LM35 <= 31) {
      servoAngle = map(int(TempC_LM35), 30, 31, 70, 80);
    } else if (TempC_LM35 <= 32) {
      servoAngle = map(int(TempC_LM35), 31, 32, 80, 90);
    } else if (TempC_LM35 <= 33) {
      servoAngle = map(int(TempC_LM35), 32, 33, 90, 100);
    } else if (TempC_LM35 <= 34) {
      servoAngle = map(int(TempC_LM35), 33, 34, 100, 110);
    } else if (TempC_LM35 <= 35) {
      servoAngle = map(int(TempC_LM35), 34, 35, 110, 120);
    } else if (TempC_LM35 <= 36) {
      servoAngle = map(int(TempC_LM35), 35, 36, 120, 130);
    } else if (TempC_LM35 <= 37) {
      servoAngle = map(int(TempC_LM35), 36, 37, 130, 140);
    } else if (TempC_LM35 <= 38) {
      servoAngle = map(int(TempC_LM35), 37, 38, 140, 150);
    } else if (TempC_LM35 <= 39) {
      servoAngle = map(int(TempC_LM35), 38, 39, 150, 160);
    } else if (TempC_LM35 <= 40) {
      servoAngle = map(int(TempC_LM35), 39, 40, 160, 170);
    } else if (TempC_LM35 <= 41) {
      servoAngle = map(int(TempC_LM35), 40, 41, 170, 180);
    } else {
      servoAngle = 180;
    }

    servoAngle = constrain(servoAngle, 0, 180); // Limitar el ángulo del servo entre 0 y 180 grados
    ledcWrite(3, servoAngle); // Mover el servo gradualmente hacia el ángulo calculado

    if (!temperatureUpdated) {
      Serial.print("Temperatura: ");
      Serial.print(TempC_LM35);
      Serial.print(" °C, Ángulo: ");
      Serial.print(servoAngle);
      Serial.println("°");
      temperatureUpdated = true;
    }

    if (TempC_LM35 < 37.0) {
      ledcWrite(LEDC_CHANNEL_0, 0);     // Apagar LED rojo
      ledcWrite(LEDC_CHANNEL_1, 255);   // Encender LED verde
      ledcWrite(LEDC_CHANNEL_2, 0);     // Apagar LED azul
    } else if (TempC_LM35 >= 37.0 && TempC_LM35 <= 37.5) {
      ledcWrite(LEDC_CHANNEL_0, 255);   // Encender LED rojo
      ledcWrite(LEDC_CHANNEL_1, 255);   // Encender LED verde
      ledcWrite(LEDC_CHANNEL_2, 0);     // Apagar LED azul
    } else {
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
    int SNLM35_Raw = analogRead(SNLM35);
    float Voltage = readADC_Cal(SNLM35_Raw);
    TempC_LM35 = ((Voltage/4095)*3.3)/0.01;

    temp = TempC_LM35 * 100;

    placeValuesofTemp[3] = ((temp) / 1) % 10;
    placeValuesofTemp[2] = ((temp) / 10) % 10;
    placeValuesofTemp[1] = ((temp) / 100) % 10;
    placeValuesofTemp[0] = ((temp) / 1000) % 10;

    temperatureTaken = true;
  }
}
