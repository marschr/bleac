#include <Arduino.h>

#define OFFSETS 350

int IRpin = 5;
unsigned long Timings[OFFSETS];

void setup() {
  Serial.begin(115200);
  Serial.println("Analyze and Decode Samsung IR Remote");
  pinMode(IRpin, INPUT);
}

void loop() {
  Serial.print("\nWaiting remcon signal...");

  unsigned int offset_index = 0;
  byte OldState = HIGH; // hi logic while ir receiver sleeps
  byte NewState;
  unsigned long StartTime, CurrentTime, PrevTime;
  bool Finished = false;
  while (true) {
    // hold first on HIGH
    while (digitalRead(IRpin) == HIGH && offset_index == 0) {
      // Serial.print("\nMicrosB4\t");
      // Serial.print(micros());
      StartTime = micros(); // start the counter
      OldState = LOW;
    }

    while (OldState == (NewState = digitalRead(IRpin))) {
      CurrentTime = micros();
      if (CurrentTime - StartTime > 500000) { // wait for more codes timeout
        Finished = true;
        break;
      }
    }
    Timings[offset_index] = CurrentTime - PrevTime; // write the delta t to
                                                    // array
    PrevTime = CurrentTime;
    if (Finished)
      break;
    offset_index++;
    OldState = NewState;
  }

  // Serial.print("\nMicros Diff: ");
  // Serial.print(micros() - StartTime);
  Serial.print("\n===>>Bit stream detected<===!\n");

  byte word[21] = {0}; // samsung AC sends 3 chunks of 7 bytes each
  unsigned int bit_index, byte_index, shifter;
  bit_index = 0;
  byte_index = 0;
  unsigned int checksum_ones_sum[3] = {0};

  for (size_t i = 0; i < OFFSETS; i++) {
    if (bit_index == 8) {
      bit_index = 0;
      byte_index++;
    }
    shifter = 7 - bit_index;

    if (Timings[i] > 8000 ||
        Timings[i] + Timings[i + 1] > 2250) { // start headers
      // Serial.print("\nhdr: \n");
      bit_index = 0;
      // detect mid headers
      if (Timings[i + 2] > 2000 && Timings[i + 3] > 7000) {
        // Serial.print("\nmidhdr: \n");
      }
    }

    // tons of serial debug stuff
    Serial.print("\n");
    Serial.print(i);
    Serial.print("i= ");
    Serial.print(Timings[i]);
    Serial.print("+");
    Serial.print(Timings[i + 1]);
    // Serial.print("us");
    // Serial.print(byte_index);
    // Serial.print("<<");
    // Serial.print(shifter);
    // Serial.print(": \t bit_index:");
    // Serial.print(bit_index);
    // Serial.print("\t bit: ");

    if (Timings[i] + Timings[i + 1] < 2500 &&
        Timings[i] + Timings[i + 1] > 1200) { // decode bit 0
      // Serial.print("0");
      word[byte_index] = word[byte_index] | (0x0 << shifter);

      bit_index++;
    }
    if (Timings[i] + Timings[i + 1] > 800 &&
        Timings[i] + Timings[i + 1] < 1200) { // decode bit 1
      // Serial.print("1");
      word[byte_index] = word[byte_index] | (0x1 << shifter);

      if (!(byte_index == 1 || byte_index == 8 ||
            byte_index == 15)) { // exclude the checksum byte itself
        // Serial.print("\t ones_sum at: ");
        // Serial.print(byte_index / 7);
        // Serial.print(" checksum_ones_sum++");
        checksum_ones_sum[byte_index / 7]++;
      }

      bit_index++;
    }
    i++;
  };

  int output_mode = HEX; // HEX or BIN
  for (size_t i = 0; i < sizeof(word); i++) {
    if (i == 0 || i == 7 || i == 14) {
      if (output_mode == HEX) {
        Serial.print("\nword: 0x");
      } else {
        Serial.print("\nword: 0b ");
      }
    }
    Serial.print(word[i], output_mode);
    Serial.print(" ");
  }

  for (size_t i = 0; i < 3; i++) {
    ouput_checksum(checksum_ones_sum[i]);
  }

  for (size_t i = 0; i < OFFSETS; i++) { // reset the timings array
    Timings[i] = (char)0;
  }

  Serial.print("\n\nBit stream end!");
  delay(500);
}

void ouput_checksum(unsigned int ones_sum) {
  Serial.print("\nones count: ");
  Serial.print(ones_sum);

  byte checksum = ones_sum % 15;
  if (checksum == 0) {
    checksum = 15;
  }
  Serial.print("\t mod15: 0b");
  Serial.print(checksum, BIN);

  checksum = ~checksum;
  Serial.print("\t flip: 0b");
  Serial.print(checksum & 0XF, BIN);

  checksum = (checksum & 0xF0) >> 4 | (checksum & 0x0F) << 4;
  checksum = (checksum & 0xCC) >> 2 | (checksum & 0x33) << 2;
  checksum = (checksum & 0xAA) >> 1 | (checksum & 0x55) << 1;

  checksum = (checksum & 0XF0) >> 4; // trim only 4 significant bits

  Serial.print("\t rev: 0b");
  Serial.print(checksum, BIN);
  Serial.print("\t rev: 0x");
  Serial.print(checksum, HEX);
}
