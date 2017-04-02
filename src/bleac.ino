#include <Stepper.h>
#include <ACS712.h>

//STEPPER MOTOR SETTINGS
#define PIN_COIL_A                        8    // the pin connected to the wire A of the coil A (or to the H-bridge pin controlling the same wire)
#define PIN_COIL_A_bar                    4    // the pin connected to the wire A- of the coil A (or to the H-bridge pin controlling the same wire)
#define PIN_COIL_B                        6    // the pin connected to the wire B of the coil A (or to the H-bridge pin controlling the same wire)
#define PIN_COIL_B_bar                    7    // the pin connected to the wire B- of the coil A (or to the H-bridge pin controlling the same wire)
#define TORQUE_SETTLE_TIME                3000 // smaller values may make the motor produce more speed and less torque
#define STEPS_PER_REV                     700  // you can the number of steps required to make a complete revolution in the data sheet of your motor

//INFRARED SETTINGS
#define PIN_IR_READ                       5
#define PIN_MODE_IR_READ                  INPUT
#define IR_SAMPLES                        350

//PWM TSENSE EMULATOR
#define PIN_TSENSE_EMULATOR_CASE          9
#define PIN_TSENSE_EMULATOR_HEXCHANGER    10
#define PIN_MODE_TSENSE_EMULATOR          OUTPUT

//DIGITAL DS18B20 TEMPERATURE SENSOR SETTINGS
#define PIN_TSENSE_DS18B20_PARASITE       11
// #define ONEW_ADDR_TSENSE_DS18B20_OUTDOOR
// #define ONEW_ADDR_TSENSE_DS18B20_CASE
// #define ONEW_ADDR_TSENSE_DS18B20_HEXCHANGER

//FAN SPEED SETTINGS
#define PIN_FAN_HALL                               12
#define PIN_MODE_FAN_HALL                          INPUT
#define FAN_HALL_REV_STEPS                         12

//ACS712 WATT SENSE
#define PIN_ANALOG_ACS712                          A1
#define AC_LINE_HERTZ                              60

//ANALOG TEMPERATURE SENSORS SETTINGS
#define PIN_ANALOG_TSENSE_INPUT_CASE               A2
#define PIN_ANALOG_TSENSE_INPUT_HEXCHANGER         A3
#define PIN_MODE_ANALOG_TSENSE_INPUT_CASE          INPUT
#define PIN_MODE_ANALOG_TSENSE_INPUT_HEXCHANGER    INPUT
int current_pin = PIN_ANALOG_TSENSE_INPUT_CASE;

bool   debug = true;
ACS712 sensor(ACS712_20A, PIN_ANALOG_ACS712);

void setup() {
    if (debug) Serial.begin(9600);
    //set all pins

    if (debug) Serial.println("setting up pin modes");

    //TODO: use ports registers
    pinMode(PIN_FAN_HALL, PIN_MODE_FAN_HALL);                                             //fan speed pin setup
    pinMode(PIN_IR_READ, PIN_MODE_IR_READ);                                               //infrared pin setup
    pinMode(PIN_ANALOG_TSENSE_INPUT_CASE, PIN_MODE_ANALOG_TSENSE_INPUT_CASE);             //analog case tsense pin setup
    pinMode(PIN_ANALOG_TSENSE_INPUT_HEXCHANGER, PIN_MODE_ANALOG_TSENSE_INPUT_HEXCHANGER); //analog HEx tsense pin setup
    pinMode(PIN_TSENSE_EMULATOR_CASE, PIN_MODE_TSENSE_EMULATOR);                          // tsense emulator case pin setup
    pinMode(PIN_TSENSE_EMULATOR_HEXCHANGER, PIN_MODE_TSENSE_EMULATOR);                    // tsense emulator hex pin setup
    sensor.calibrate();
    if (debug) Serial.println("setup() done.");
}


void loop() {
    //1 - Init config/settings
    //2 - Read temps
    //3 - Write analog tsense emulator
    //4 - wait for AC mainboard to move stepper to startup position
    //4.1 - move stepper to set initial position
    //5 - read everything
    //5.1 - fan speed
    //5.2 - analog tsensors
    //5.3 - ds18b20s
    //5.4 - recalculate pwm duty based on virtual circuit + integral
    //      approx of temp over time due to external temps
    //5.5 - update analog tsense emulator
    //5.6 - read ACS power
    //5.7 - accumulate data
    //6 - move stepper to next endpoint
    //7 - jump to 5, wait for BLE/IR interrupt with new settings


    if (debug) Serial.print("loop() read_fan_speed(): ");
    read_fan_speed();
    read_analog_tsenses();
    read_db18s20();
}

void read_db18s20(){

}

void stepper_flap() {
    //TODO: define protocol modes: full up, full down, angle, swing from angle to angle;
    //TODO:
    //if (startangle == endangle) {move to pos; goto read loop}
    // else { move to posA; goto read loop; move to posB}

    //Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);
}


void read_acs712() {
    float U = 220;
    float I = sensor.getCurrentAC(AC_LINE_HERTZ);

    // To calculate the power we need voltage multiplied by current
    float P = U * I;

    Serial.println(String("I = ") + I + " A");
    Serial.println(String("P = ") + P + " Watts");
}


int calculate_pwm_duty() {
    //higher values = lower temps

    //needed fields:
    // outdoor temps
    // case and heat exchanger thermistor ohms val
    // ds18b20s temperature vals
    // coarsing values for integral calculation
    // virtual circuit ohm resitor values

    int caseVal      = 170;
    int exchangerVal = 160;

    analogWrite(PIN_TSENSE_EMULATOR_CASE, caseVal);
    analogWrite(PIN_TSENSE_EMULATOR_HEXCHANGER, exchangerVal);

    //TODO: calculate it for real
    return 1;
}


void read_analog_tsenses() {
    int   raw    = 0;
    float Vin    = 4.73;
    float Vout   = 0;
    float R1     = 1000;
    float R2     = 0;
    float buffer = 0;

    if (current_pin == PIN_ANALOG_TSENSE_INPUT_CASE) {
        raw         = analogRead(PIN_ANALOG_TSENSE_INPUT_CASE);
        current_pin = PIN_ANALOG_TSENSE_INPUT_HEXCHANGER; //swap
        Serial.print("Case ");
    }
    else {
        raw         = analogRead(PIN_ANALOG_TSENSE_INPUT_HEXCHANGER);
        current_pin = PIN_ANALOG_TSENSE_INPUT_CASE; //swap back
        Serial.print("Exchanger ");
    }

    if (raw) {
        Serial.print("raw: ");
        Serial.print(raw);
        buffer = raw * Vin;
        Vout   = (buffer) / 1024.0;
        buffer = (Vin / Vout) - 1;
        R2     = R1 * buffer;
        Serial.print("\tVout: ");
        Serial.print(Vout);
        Serial.print("\tR2: ");
        Serial.println(R2);
    }
}


//decode_ir() BEGIN!
void decode_ir() {
    unsigned long Timings[IR_SAMPLES];

    if (debug) Serial.print("\nWaiting remcon signal...");

    unsigned int  offset_index = 0;
    byte          OldState     = HIGH; // hi logic while ir receiver sleeps
    byte          NewState;
    unsigned long StartTime, CurrentTime, PrevTime;
    bool          Finished = false;
    while (true) {
        // hold first on HIGH
        while (digitalRead(PIN_IR_READ) == HIGH && offset_index == 0) {
            // Serial.print("\nMicrosB4\t");
            // Serial.print(micros());
            StartTime = micros(); // start the counter
            OldState  = LOW;
        }

        while (OldState == (NewState = digitalRead(PIN_IR_READ))) {
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
    if (debug) Serial.print("\n===>>Bit stream detected<===!\n");

    byte         word[21] = { 0 }; // samsung AC sends 3 chunks of 7 bytes each
    unsigned int bit_index, byte_index, shifter;
    bit_index  = 0;
    byte_index = 0;
    unsigned int checksum_ones_sum[3] = { 0 };

    for (size_t i = 0; i < IR_SAMPLES; i++) {
        if (bit_index == 8) {
            bit_index = 0;
            byte_index++;
        }
        shifter = 7 - bit_index;

        if ((Timings[i] > 8000) ||
            (Timings[i] + Timings[i + 1] > 2250)) { // start headers
            // Serial.print("\nhdr: \n");
            bit_index = 0;
            // detect mid headers
            if ((Timings[i + 2] > 2000) && (Timings[i + 3] > 7000)) {
                // Serial.print("\nmidhdr: \n");
            }
        }

        // tons of serial debug stuff
        if (debug) {
            Serial.print("\n");
            Serial.print(i);
            Serial.print("i= ");
            Serial.print(Timings[i]);
            Serial.print("+");
            Serial.print(Timings[i + 1]);
        }
        // Serial.print("us");
        // Serial.print(byte_index);
        // Serial.print("<<");
        // Serial.print(shifter);
        // Serial.print(": \t bit_index:");
        // Serial.print(bit_index);
        // Serial.print("\t bit: ");

        if ((Timings[i] + Timings[i + 1] < 2500) &&
            (Timings[i] + Timings[i + 1] > 1200)) { // decode bit 0
            // Serial.print("0");
            word[byte_index] = word[byte_index] | (0x0 << shifter);

            bit_index++;
        }
        if ((Timings[i] + Timings[i + 1] > 800) &&
            (Timings[i] + Timings[i + 1] < 1200)) { // decode bit 1
            // Serial.print("1");
            word[byte_index] = word[byte_index] | (0x1 << shifter);

            if (!((byte_index == 1) || (byte_index == 8) ||
                  (byte_index == 15))) { // exclude the checksum byte itself
                // Serial.print("\t ones_sum at: ");
                // Serial.print(byte_index / 7);
                // Serial.print(" checksum_ones_sum++");
                checksum_ones_sum[byte_index / 7]++;
            }

            bit_index++;
        }
        i++;
    }

    int output_mode = HEX; // HEX or BIN
    for (size_t i = 0; i < sizeof(word); i++) {
        if ((i == 0) || (i == 7) || (i == 14)) {
            if (debug) {
                if (output_mode == HEX) {
                    Serial.print("\nword: 0x");
                }
                else {
                    Serial.print("\nword: 0b ");
                }
            }
        }
        Serial.print(word[i], output_mode);
        Serial.print(" ");
    }

    for (size_t i = 0; i < 3; i++) {
        ouput_checksum(checksum_ones_sum[i]);
    }

    for (size_t i = 0; i < IR_SAMPLES; i++) { // reset the timings array
        Timings[i] = (char)0;
    }

    if (debug) Serial.print("\n\nBit stream end!");
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


//decode_ir() END!

//read_fan_speed() BEGIN!
int read_fan_speed() {
    byte state, OldState;
    int  steps    = 0;
    int  revSteps = FAN_HALL_REV_STEPS;
    bool Finished = false;

    state = digitalRead(PIN_FAN_HALL);           //**state = low
    unsigned long StartTime, CurrentTime, PrevTime;
    while (digitalRead(PIN_FAN_HALL) == state) { //hold while low
                                                 // hold here while it doesnt change state/rev
        StartTime = micros();                    //**keep updating start counter
        OldState  = state;                       //** keep last state updated
    }
    if (debug) Serial.print("\ninit_step");

    while (true) {
        OldState = digitalRead(PIN_FAN_HALL);
        while (digitalRead(PIN_FAN_HALL) == OldState) {
            CurrentTime = micros();
            if (CurrentTime - StartTime > 1000000) {         // wait 1s
                Finished = true;
                break;
            }
        }
        steps++;

        if (Finished) {
            break;
        }
    }
    if (debug) {
        Serial.print("Loop end, steps: ");
        Serial.print(steps, DEC);

        Serial.print("\tRPM: ");
        Serial.print((steps / revSteps) * 60, DEC);

        Serial.print("\tRPMstep: ");
        Serial.println(steps * 5, DEC);
    }

    return 1;
}
