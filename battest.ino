#define A_PIN 1
#define NUM_READS 100
#define pinRelay 7 // pin for discharger
#define pinCharge 6 // pin for charger
#define pinBlink 13 // pin for blinker
#define COND 496   // Shunt conductivity (mS), =1000/Rshunt. Defines by hardware.
#define VBG 1.095  // Real reference voltage. Defines by hardware.
#define CCURR 180  // Charge current. Defines by hardware.
#define DV 0.006   // Delta voltage for voltage peak detection, less=better but more sensitive to ADC noise
#define THRES 1.9  // Charging threshold, Volts (charging stops when cell voltage exceeds the value)

bool charging;
float Voff = 0.8;  // Voltage when discharging ends.
float I;
float deltacap;
float cap = 0;
float V;
float Vcc;
float Wh = 0;
float chp[3] = { 0.0 }; // Charge-discharge parameters
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
  // Start charging...
  digitalWrite(pinCharge, HIGH);
  delay(4);
  if (((analogRead(A_PIN) * 5.0) / 1024.0) > THRES) {
    Serial.println("No or bad battery");  //ERROR Voltage beyond threshold. Charging will be stopped.
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
  // Discharging...
  digitalWrite(pinRelay, HIGH);
  charging = true;
  Serial.println("Test is launched...");
  Serial.println("s V mA mAh Wh Ri");

  testStart = millis();
  prevMillis = testStart;
  offTime = 0;
  do {
    Vcc = (VBG * 1024.0) / readAnalog(-1);     // Get reference voltage
    V = (readAnalog(A_PIN) * Vcc) / 1024.000;  // Get cell voltage
    I = COND * V; //current calculating
    deltacap = I * (millis() - prevMillis - offTime) / 3600000000;
    prevMillis = millis();
    offTime = 0;
    cap += deltacap * 1000;  // Accumulated cell capacity mAh
    Wh += deltacap * V;      // Accumulated cell capacity Wh
    Ri = measureRi(1); // Cell internal resistance

    sendData(prevMillis - testStart);  // Sending data to terminal
  } while (V > Voff); //Do discharging while cell voltage exceeds Voff
  digitalWrite(pinRelay, LOW);
  charging = false;
  Serial.println("Test is done");
  serFlush();
  // End of testing.
  while (true) {
    // Print result to terminal by key strike...
    if (Serial.available()) {
      Serial.println("s V mA mAh Wh Ri");
      sendData(prevMillis - testStart);
      sendChData();
      sendRi(Ristart);
      serFlush();
    }
    // Blinker processing
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
	//Charge control.
	// Process breaks after 'time' millisecs
	// or if the voltage reaches THRES
	// also gets some statistics about process
	// (time when cell voltage reaches its maximum
	// and maximum voltage value)
  unsigned long dlyStart = millis();
  serFlush();
  while (millis() - dlyStart < time) {
    Vcc = (VBG * 1024.0) / readAnalog(-1);  // Gets reference voltage
    if (Serial.available()) break;
    V = (readAnalog(A_PIN) * Vcc) / 1024.000;  // Get cell voltage
    if (V > THRES) break;                      // ERROR Voltage beyond threshold.
    if (V - chp[1] >= DV) {
      chp[1] = V;
      chp[0] = (millis() - dlyStart) / 1000;
    }
  }
  chp[2] = (millis() - dlyStart) / 1000;
  return;
}