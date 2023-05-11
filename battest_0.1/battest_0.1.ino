#define A_PIN 1
#define NUM_READS 100
#define pinRelay 7

const float typVbg = 1.095; // 1.0 -- 1.2
float Voff = 2.5; // напряжение выключения
float I;
float cap = 0;
float V;
float Vcc;
float Wh = 0;
unsigned long prevMillis;
unsigned long testStart;

void setup() {
  Serial.begin(9600);
  pinMode(pinRelay, OUTPUT);
  Serial.println("Press any key to start the test...");
  while (Serial.available() == 0) {
  }
  Serial.println("Test is launched...");
  Serial.print("s");
  Serial.print(" ");
  Serial.print("V");
  Serial.print(" ");
  Serial.print("mA");
  Serial.print(" ");
  Serial.print("mAh");
  Serial.print(" ");
  Serial.print("Wh");
  Serial.print(" ");
  Serial.println("Vcc");
  digitalWrite(pinRelay, HIGH);
  testStart = millis();
  prevMillis = millis();
}

void loop() {
  Vcc = readVcc(); //считывание опорного напряжения
  V = (readAnalog(A_PIN) * Vcc) / 1023.000; //считывание напряжения АКБ
  if (V > 0.01) I = -13.1 * V * V + 344.3 * V + 23.2; //расчет тока по ВАХ спирали
  else I=0;
  cap += (I * (millis() - prevMillis) / 3600000); //расчет емкости АКБ в мАч
  Wh += I * V * (millis() - prevMillis) / 3600000000; //расчет емкости АКБ в ВтЧ
  prevMillis = millis();
  sendData(); // отправка данных в последовательный порт
  if (V < Voff) { //выключение нагрузки при достижении порогового напряжения
    digitalWrite(pinRelay, LOW);
    Serial.println("Test is done");
    while (2 > 1) {
    }
  }
}

void sendData() {
  Serial.print((millis() - testStart) / 1000);
  Serial.print(" ");
  Serial.print(V, 3);
  Serial.print(" ");
  Serial.print(I, 1);
  Serial.print(" ");
  Serial.print(cap, 0);
  Serial.print(" ");
  Serial.print(Wh, 2);
  Serial.print(" ");
  Serial.println(Vcc, 3);
}


float readAnalog(int pin) {
  // read multiple values and sort them to take the mode
  int sortedValues[NUM_READS];
  for (int i = 0; i < NUM_READS; i++) {
    delay(25);
    int value = analogRead(pin);
    int j;
    if (value < sortedValues[0] || i == 0) {
      j = 0; //insert at first position
    }
    else {
      for (j = 1; j < i; j++) {
        if (sortedValues[j - 1] <= value && sortedValues[j] >= value) {
          // j is insert position
          break;
        }
      }
    }
    for (int k = i; k > j; k--) {
      // move all values higher than current reading up one position
      sortedValues[k] = sortedValues[k - 1];
    }
    sortedValues[j] = value; //insert current reading
  }
  //return scaled mode of 10 values
  float returnval = 0;
  for (int i = NUM_READS / 2 - 5; i < (NUM_READS / 2 + 5); i++) {
    returnval += sortedValues[i];
  }
  return returnval / 10;
}


float readVcc() {
  // read multiple values and sort them to take the mode
  float sortedValues[NUM_READS];
  for (int i = 0; i < NUM_READS; i++) {
    float tmp = 0.0;
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    ADCSRA |= _BV(ADSC); // Start conversion
    delay(25);
    while (bit_is_set(ADCSRA, ADSC)); // measuring
    uint8_t low = ADCL; // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both
    tmp = (high << 8) | low;
    float value = (typVbg * 1023.0) / tmp;
    int j;
    if (value < sortedValues[0] || i == 0) {
      j = 0; //insert at first position
    }
    else {
      for (j = 1; j < i; j++) {
        if (sortedValues[j - 1] <= value && sortedValues[j] >= value) {
          // j is insert position
          break;
        }
      }
    }
    for (int k = i; k > j; k--) {
      // move all values higher than current reading up one position
      sortedValues[k] = sortedValues[k - 1];
    }
    sortedValues[j] = value; //insert current reading
  }
  //return scaled mode of 10 values
  float returnval = 0;
  for (int i = NUM_READS / 2 - 5; i < (NUM_READS / 2 + 5); i++) {
    returnval += sortedValues[i];
  }
  return returnval / 10;
}