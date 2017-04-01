#define CASE_TERMISTOR_PIN A2
#define EXCHANGER_TERMISTOR_PIN A3

int raw = 0;
float Vin = 4.73;
float Vout = 0;
float R1 = 1000;
float R2 = 0;
float buffer = 0;

int current_pin;

void setup() {
  current_pin = CASE_TERMISTOR_PIN;
  Serial.begin(9600); }

void loop() {

  if (current_pin == CASE_TERMISTOR_PIN) {
    raw = analogRead(CASE_TERMISTOR_PIN);
    current_pin = EXCHANGER_TERMISTOR_PIN; //swap
    Serial.print("Case ");
  } else {
    raw = analogRead(EXCHANGER_TERMISTOR_PIN);
    current_pin = CASE_TERMISTOR_PIN; //swap back
    Serial.print("Exchanger ");
  }

  if (raw) {
    Serial.print("raw: ");
    Serial.print(raw);
    buffer = raw * Vin;
    Vout = (buffer) / 1024.0;
    buffer = (Vin / Vout) - 1;
    R2 = R1 * buffer;
    Serial.print("\tVout: ");
    Serial.print(Vout);
    Serial.print("\tR2: ");
    Serial.println(R2);
    delay(1000);
  }
}
