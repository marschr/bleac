void setup(void) {
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  Serial.begin(9600);
  Serial.println("DAC Setup...");
}

int caseVal = 170;
int exchangerVal = 160;

void loop(void) {
  analogWrite(9, caseVal);
  analogWrite(10, exchangerVal);

}
