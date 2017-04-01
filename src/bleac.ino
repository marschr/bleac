#include <Arduino.h>

int IRpin = 3;

void setup() {
  Serial.begin(9600);
  Serial.println("Read Samsung AC Fan Hall Speed");
  pinMode(IRpin, INPUT);
}

void loop() {
  byte state, OldState;
  int steps = 0;
  int revSteps = 12;
  bool Finished = false;
  state = digitalRead(IRpin); //**state = low

  unsigned long StartTime, CurrentTime, PrevTime;

  while (digitalRead(IRpin) == state) { //**hold while low
    // hold here while it doesnt change state/rev
    StartTime = micros(); //**keep updating start counter
    OldState = state;     //** keep last state updated
  }
  Serial.print("\ninit_step");

  while (true) {
    OldState = digitalRead(IRpin);
    // for (size_t i = 0; i < revSteps; i++) {
    while (digitalRead(IRpin) == OldState) {
      CurrentTime = micros();
      if (CurrentTime - StartTime > 1000000) { // wait 1s
        Finished = true;
        break;
      }
      // OldState = digitalRead(IRpin);
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
