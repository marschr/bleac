#include <Arduino.h>

int FAN_PIN = 12;

void setup() {
  Serial.begin(9600);
  Serial.println("Read Samsung AC Fan Hall Speed");
  pinMode(FAN_PIN, INPUT);
}

void loop() {
  byte state, OldState;
  int steps = 0;
  int revSteps = 12;
  bool Finished = false;
  state = digitalRead(FAN_PIN); //**state = low

  unsigned long StartTime, CurrentTime, PrevTime;

  while (digitalRead(FAN_PIN) == state) { //**hold while low
    // hold here while it doesnt change state/rev
    StartTime = micros(); //**keep updating start counter
    OldState = state;     //** keep last state updated
  }
  Serial.print("\ninit_step");

  while (true) {
    OldState = digitalRead(FAN_PIN);
    // for (size_t i = 0; i < revSteps; i++) {
    while (digitalRead(FAN_PIN) == OldState) {
      CurrentTime = micros();
      if (CurrentTime - StartTime > 1000000) { // wait 1s
        Finished = true;
        break;
      }
      // OldState = digitalRead(FAN_PIN);
    }
    // Serial.print("\nstep++");
    steps++;

    if (Finished)
      break;
  }
  // i++;
  // }

  Serial.print("Loop end, steps: ");
  Serial.print(steps, DEC);

  Serial.print("\tRPM: ");
  Serial.print((steps / revSteps) * 60, DEC);

    Serial.print("\tRPMstep: ");
  Serial.println(steps*5, DEC);
}
