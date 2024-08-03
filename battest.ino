#define A_PIN 1
#define NUM_READS 100
#define pinRelay 7
#define pinCharge 6
#define pinBlink 13
#define COND 496   // проводимость шунта в мСм, =1000/Rш
#define VBG 1.095  // 1.0 -- 1.2 опорное
#define CCURR 180  //Ток зарядки
#define DV 0.006   //вольт от максимума считаются новым максимумом
#define THRES 1.9  //предельное напряжение при зарядке, выше которого заряд прекращается

bool charging;
float Voff = 0.8;  // напряжение выключения
float I;
float deltacap;
float cap = 0;
float V;
float Vcc;
float Wh = 0;
float chp[3] = { 0.0 };
float Ri;
float Ristart;
unsigned long prevMillis;
unsigned long testStart;
unsigned long offTime;

void setup() {
  Serial.begin(9600);
  pinMode(pinRelay, OUTPUT);
  digitalWrite(pinRelay, LOW);
  charging = false;
  pinMode(pinCharge, OUTPUT);
  digitalWrite(pinCharge, LOW);
  pinMode(pinBlink, OUTPUT);
  digitalWrite(pinBlink, LOW);
}

void loop() {
  //Начинаем заряд
  digitalWrite(pinCharge, HIGH);
  delay(4);
  if (((analogRead(A_PIN) * 5.0) / 1024.0) > THRES) {
    Serial.println("No battery");  //Напряжение резко выросло, видимо батарея не подключена или неисправна.
  } else {
    Serial.println("Charging. Press a key to start the test or wait for 16 hours");
    brkDlyCh(57600000);
    sendChData();
  }

  digitalWrite(pinCharge, LOW);

  Serial.println("Pause 60 s before discharging");
  brkDly(60000);

  Ristart = measureRi(5);
  Ri = Ristart;
  sendRi(Ristart);
  //Старт разряда
  digitalWrite(pinRelay, HIGH);
  charging = true;
  Serial.println("Test is launched...");
  Serial.println("s V mA mAh Wh Ri");

  testStart = millis();
  prevMillis = testStart;
  offTime = 0;
  do {
    Vcc = (VBG * 1024.0) / readAnalog(-1);     //считывание опорного напряжения
    V = (readAnalog(A_PIN) * Vcc) / 1024.000;  //считывание напряжения АКБ
    //if (V > 0.01) I = -13.1 * V * V + 344.3 * V + 23.2;  //расчет тока по ВАХ спирали
    //else I = 0;
    I = COND * V;
    deltacap = I * (millis() - prevMillis - offTime) / 3600000000;
    prevMillis = millis();
    offTime = 0;
    cap += deltacap * 1000;  //расчет емкости АКБ в мАч
    Wh += deltacap * V;      //расчет емкости АКБ в ВтЧ
    Ri = measureRi(1);

    sendData(prevMillis - testStart);  // отправка данных в последовательный порт
  } while (V > Voff);
  //выключение нагрузки при достижении порогового напряжения
  digitalWrite(pinRelay, LOW);
  charging = false;
  Serial.println("Test is done");
  serFlush();
  //Закончили тест, ожидаем считывания результата
  while (true) {
    //Есть запрос - печатаем результат
    if (Serial.available()) {
      Serial.println("s V mA mAh Wh Ri");
      sendData(prevMillis - testStart);
      sendChData();
      sendRi(Ristart);
      serFlush();
    }
    //Выводим данные измерений на мигалку
    blink((int)(cap / 100));
  }
}

void sendData(unsigned long timedelta) {
  Serial.print(timedelta / 1000);
  Serial.print(" ");
  Serial.print(V, 3);
  Serial.print(" ");
  Serial.print(I, 1);
  Serial.print(" ");
  Serial.print(cap, 0);
  Serial.print(" ");
  Serial.print(Wh, 2);
  Serial.print(" ");
  Serial.println(Ri * 1000, 2);
}

void sendChData() {
  Serial.println("ChrgT StopT --V-- mAh Charge_result");
  Serial.print(chp[2], 0);
  Serial.print(" ");
  Serial.print(chp[0], 0);
  Serial.print(" ");
  Serial.print(chp[1], 3);
  Serial.print(" ");
  Serial.println(chp[0] * CCURR / 3600, 0);
}

void sendRi(float valRi) {
  Serial.print("Rinternal=");
  Serial.print(1000 * valRi);
  Serial.println(" mOhm");
}

float measureRi(int cnt) {
  float Udelta = 0.0;
  float Uonsum = 0.0;
  float Result;
  int Uoff;
  int Uon;
  for (int i = 0; i < cnt; i++) {
    delay(16);
    if (charging) {
      Uon = analogRead(A_PIN);
      digitalWrite(pinRelay, LOW);
      delay(16);
      Uoff = analogRead(A_PIN);
      digitalWrite(pinRelay, HIGH);
      offTime += 16;
    } else {
      Uoff = analogRead(A_PIN);
      digitalWrite(pinRelay, HIGH);
      delay(16);
      Uon = analogRead(A_PIN);
      digitalWrite(pinRelay, LOW);
    }
    Udelta += (Uoff - Uon);
    Uonsum += Uon;
  }
  if (Uonsum == 0) Result = 0;
  else Result = 1000 * Udelta / (Uonsum * COND);
  return Result;
}

float readAnalog(int pin) {
  // read multiple values and sort them to take the mode
  int sortedValues[NUM_READS];
  int value;
  for (int i = 0; i < NUM_READS; i++) {
    if (pin < 0) {
      ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
      ADCSRA |= _BV(ADSC);  // Start conversion
      delay(25);
      while (bit_is_set(ADCSRA, ADSC))
        ;                   // measuring
      uint8_t low = ADCL;   // must read ADCL first - it then locks ADCH
      uint8_t high = ADCH;  // unlocks both
      value = (high << 8) | low;
    } else {
      delay(25);
      value = analogRead(pin);
    }
    int j;
    if (value < sortedValues[0] || i == 0) {
      j = 0;  //insert at first position
    } else {
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
    sortedValues[j] = value;  //insert current reading
  }
  //return scaled mode of 10 values
  float returnval = 0;
  for (int i = NUM_READS / 2 - 5; i < (NUM_READS / 2 + 5); i++) {
    returnval += sortedValues[i];
  }
  return returnval / 10;
}

void blink(int n) {
  if (n > 100) n = 100;
  while (n > 0) {
    if (n > 9) {
      digitalWrite(pinBlink, HIGH);
      n -= 10;
      delay(625);
    } else {
      digitalWrite(pinBlink, HIGH);
      n -= 1;
      delay(125);
    }
    digitalWrite(pinBlink, LOW);
    delay(375);
  }
  delay(500);
}

void serFlush() {
  while (Serial.available()) {
    while (Serial.available()) Serial.read();
    delay(25);
  }
}

void brkDly(unsigned long time) {
  unsigned long dlyStart = millis();
  serFlush();
  while (millis() - dlyStart < time) {
    if (Serial.available()) break;
  }
  return;
}

void brkDlyCh(unsigned long time) {
  unsigned long dlyStart = millis();
  serFlush();
  while (millis() - dlyStart < time) {
    Vcc = (VBG * 1024.0) / readAnalog(-1);  //считывание опорного напряжения
    if (Serial.available()) break;
    V = (readAnalog(A_PIN) * Vcc) / 1024.000;  //считывание напряжения АКБ
    if (V > THRES) break;                      //Аккумулятор не подключен или очень плох.
    if (V - chp[1] >= DV) {
      chp[1] = V;
      chp[0] = (millis() - dlyStart) / 1000;
    }
  }
  chp[2] = (millis() - dlyStart) / 1000;
  return;
}