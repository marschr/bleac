#include <Arduino.h>
#include <Stepper.h>
#include <ACS712.h>
#include <OneWire.h>
#include <Stepper.h>


//STEPPER MOTOR SETTINGS
#define PIN_STEPPER_COIL_A                  6    // the pin connected to the wire A of the coil A (or to the H-bridge pin controlling the same wire)
#define PIN_STEPPER_COIL_A_bar              4    // the pin connected to the wire A- of the coil A (or to the H-bridge pin controlling the same wire)
#define PIN_STEPPER_COIL_B                  8    // the pin connected to the wire B of the coil A (or to the H-bridge pin controlling the same wire)
#define PIN_STEPPER_COIL_B_bar              7    // the pin connected to the wire B- of the coil A (or to the H-bridge pin controlling the same wire)
#define TORQUE_SETTLE_TIME                3000 // smaller values may make the motor produce more speed and less torque
#define STEPS_PER_REV                      700  // you can the number of steps required to make a complete revolution in the data sheet of your motor

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
#define PIN_ANALOG_TSENSE_INPUT_CASE               A3
#define PIN_ANALOG_TSENSE_INPUT_HEXCHANGER         A2
#define PIN_MODE_ANALOG_TSENSE_INPUT_CASE          INPUT
#define PIN_MODE_ANALOG_TSENSE_INPUT_HEXCHANGER    INPUT

bool   debug = true;

//Thermistor temperature to ohms table, from -9 celsius to +50 celsius (10kohms at 25C) (103AT Thermistor)
unsigned int ThermistorArray[60] = {
    40560, 38760, 37050, 35430, 33890, 32430, 31040, 29720, 28470, 27280, // -9 to 0
    26130, 25030, 23900, 22990, 22050, 21150, 20290, 19480, 18700, 17960, // 1 to 10
    17240, 16550, 15900, 15280, 14680, 14120, 13570, 13060, 12560, 12090, // 11 to 20
    11630, 11200, 10780, 10380, 10000,  9632,  9281,  8944,  8622,  8313, // 21 to 30
    8015,   7729,  7455,  7192,  6941,  6699,  6468,  6246,  6033,  5828, // 31 to 40
    5630,   5439,  5256,  5080,  4912,  4749,  4594,  4444,  4300,  4161};// 41 to 50

//Result from temps array is an int with XX.XX celcius (two decimal points)
int tempsFromArray(unsigned int ohms){
    int resultTemp;
    size_t isize = sizeof(ThermistorArray) / sizeof(ThermistorArray[0]);
    for (size_t i = 0; i < isize - 1; i++) //iterate for values of Thermistor array -1 = 59;
    {
        if (ThermistorArray[i] >= ohms && ohms > ThermistorArray[i+1]) {
            // if (debug) Serial.print("index[");
            // if (debug) Serial.print(i, DEC);
            // if (debug) Serial.print("]: ");
            // if (debug) Serial.print(ThermistorArray[i]);
            // if (debug) Serial.print(" temp from table: ");
            // if (debug) Serial.println((int)i-9, DEC);

            //get the range between the found table temp and the next
            int range = ThermistorArray[i] - ThermistorArray[i+1];
            // if (debug) Serial.print("range->");
            // if (debug) Serial.print(range, DEC);

            //calculate how many ohms above the next (hotter) table temp
            long val = ohms - ThermistorArray[i+1];
            // if (debug) Serial.print(" val->");
            // if (debug) Serial.println(val, DEC);
            
            //use map to interpolate the decimal range
            int decimals = map(val, range, 0, 0, 100);
            // if (debug) Serial.println(decimals, DEC);

            resultTemp = ((int)i-9)*100 + decimals;
            break;
        }
    }
    // if (debug) Serial.print("result: ");
    // if (debug) Serial.println(resultTemp);
    return resultTemp;
}

//ACS712 lib init
ACS712 sensor(ACS712_20A, PIN_ANALOG_ACS712);

//DS18x20 pin and init
OneWire  ds(11);

//Stepper flap init
Stepper myStepper(STEPS_PER_REV, PIN_STEPPER_COIL_A, PIN_STEPPER_COIL_A_bar, PIN_STEPPER_COIL_B, PIN_STEPPER_COIL_B_bar);


void read_db18s20(){
    byte i;
    byte present = 0;
    byte type_s;
    byte data[12];
    byte addr[8];
    float celsius, fahrenheit;

    if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
    }

    Serial.print("ROM =");
    for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
    }

    if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        return;
    }
    Serial.println();

    // the first ROM byte indicates which chip
    switch (addr[0]) {
    case 0x10:
        Serial.println("  Chip = DS18S20");  // or old DS1820
        type_s = 1;
        break;
    case 0x28:
        Serial.println("  Chip = DS18B20");
        type_s = 0;
        break;
    case 0x22:
        Serial.println("  Chip = DS1822");
        type_s = 0;
        break;
    default:
        Serial.println("Device is not a DS18x20 family device.");
        return;
    } 

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end

    delay(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.

    present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE);         // Read Scratchpad

    Serial.print("  Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
    }
    Serial.print(" CRC=");
    Serial.print(OneWire::crc8(data, 8), HEX);
    Serial.println();

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
    }
    } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
    }
    celsius = (float)raw / 16.0;
    fahrenheit = celsius * 1.8 + 32.0;
    Serial.print("  Temperature = ");
    Serial.print(celsius);
    Serial.print(" Celsius, ");
    Serial.print(fahrenheit);
    Serial.println(" Fahrenheit");
}

void stepper_flap() {

    //TODO: define protocol modes: full up, full down, angle, swing from angle to angle;
    //TODO:
    //if (startangle == endangle) {move to pos; goto read loop}
    // else { move to posA; goto read loop; move to posB}

    pinMode(PIN_STEPPER_COIL_A, OUTPUT);    
    pinMode(PIN_STEPPER_COIL_A_bar, OUTPUT);
    pinMode(PIN_STEPPER_COIL_B, OUTPUT);    
    pinMode(PIN_STEPPER_COIL_B_bar, OUTPUT);

    //Could swing at speed 8~10, tho 10 and above it may skip a few steps (thus needing to zero the position again)
    //
    myStepper.setSpeed(35);

    //Reset flat position (should be moved into init code after)
    //-1500 seems a bit too much, need to figure out proper values compatible with a non skipping speed
    Serial.println("Reseting stepper position...");
    myStepper.step(-STEPS_PER_REV-50);

    Serial.println("stepping...");

    myStepper.setSpeed(35);

    myStepper.step(STEPS_PER_REV);

    //We need to reset the pins back to INPUT so the AC controller can also controll the flap (if we really want to)
    Serial.println("resetting stepper pins...");
    pinMode(PIN_STEPPER_COIL_A, INPUT);    
    pinMode(PIN_STEPPER_COIL_A_bar, INPUT);
    pinMode(PIN_STEPPER_COIL_B, INPUT);    
    pinMode(PIN_STEPPER_COIL_B_bar, INPUT);
    
}


void read_acs712() {
    //TODO: maybe we could set a timout when the AC receives a TurnOff signal,
    //      then time a few seconds and re-run the current sensor calibration
    float U = 220;
    float I = sensor.getCurrentAC(AC_LINE_HERTZ);

    // To calculate the power we need voltage multiplied by current
    float P = U * I;

    Serial.println(String("I = ") + I + " A");
    Serial.println(String("P = ") + P + " Watts");
}


int calculate_pwm_duty(float vcc, float rther) {
    //needed fields:
    // outdoor temps DONE
    // case and heat exchanger thermistor ohms val DONE
    // ds18b20s temperature vals DONE
    // virtual circuit ohm resitor values DONE
    // coarsing values for PID calculation using outside temp

    int r1 = 6800;
    // Vout = Vcc*Rther / R1 + Rther
    float vout = ((vcc/1000) * rther) / (r1 + rther);

    Serial.print("calc vout:");
    Serial.print(vout, DEC);
    
    // duty = (255/Vcc)*Vout
    int duty = (255/(vcc/1000)) * vout;
    
    Serial.print(" calc duty:");
    Serial.println(duty, DEC);
    return duty;
}

//Reads internal Arduino VRef
long readVcc() { long result;
    // Read 1.1V reference against AVcc
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2);
    // Wait for Vref to settle
    ADCSRA |= _BV(ADSC);
    // Convert
    while (bit_is_set(ADCSRA,ADSC));
    result = ADCL;
    result |= ADCH<<8;
    result = 1126400L / result;
    // Back-calculate AVcc in mV
    return result;
 }

void read_analog_tsenses() {
    float R1 = 1000;
    float R2Case = 0;
    float R2HeatEx = 0;
    float Vin = readVcc();

    float VoutCase = 0;
    float VoutHeatEx  = 0;

    int dutyCase = 0;
    int dutyHeatEx = 0;


    //read 328p thermistor analog pins
    int rawCase = analogRead(PIN_ANALOG_TSENSE_INPUT_CASE);
    int rawHeatEx = analogRead(PIN_ANALOG_TSENSE_INPUT_HEXCHANGER);

    //find R2/termistor values for case
    VoutCase = (rawCase * Vin) / 1024;
    R2Case = R1 * (Vin / VoutCase) - 1;
    //and for heat exchanger
    VoutHeatEx = (rawHeatEx * Vin) / 1024;
    R2HeatEx = R1 * (Vin / VoutHeatEx) - 1;

    //TODO fix mess with floats and ints
    dutyCase = calculate_pwm_duty(Vin, R2Case);
    dutyHeatEx = calculate_pwm_duty(Vin, R2HeatEx);

    Serial.print("Vin ");
    Serial.println(Vin);

    Serial.print("case raw:\t");
    Serial.print(rawCase);
    Serial.print("\tVoutCase: ");
    Serial.print(VoutCase);
    Serial.print("\tR2: ");
    Serial.print(R2Case);
    Serial.print("\t103AT temp: ");
    Serial.print((float)tempsFromArray(R2Case)/100, DEC);
    Serial.print("\t duty: ");
    Serial.println(dutyCase, DEC);

    Serial.print("exchanger raw:\t");
    Serial.print(rawHeatEx);
    Serial.print("\tVoutExchanger: ");
    Serial.print(VoutHeatEx);
    Serial.print("\tR2: ");
    Serial.print(R2HeatEx);
    Serial.print("\t103AT temp: ");
    Serial.print((float)tempsFromArray(R2HeatEx)/100, DEC);
    Serial.print("\t duty: ");
    Serial.println(dutyHeatEx, DEC);

    analogWrite(PIN_TSENSE_EMULATOR_CASE, dutyCase);
    analogWrite(PIN_TSENSE_EMULATOR_HEXCHANGER, dutyHeatEx);

    //debug code with hardwired values
    // analogWrite(PIN_TSENSE_EMULATOR_CASE, 151);
    // analogWrite(PIN_TSENSE_EMULATOR_HEXCHANGER, 151);
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


//decode_ir() END!

//read_fan_speed() BEGIN!
int read_fan_speed() {
    byte state, OldState;
    int  steps    = 0;
    int  revSteps = FAN_HALL_REV_STEPS;
    bool Finished = false;

    state = digitalRead(PIN_FAN_HALL);           //**state = low
    unsigned long StartTime, CurrentTime, PrevTime;
    StartTime = micros();
    while (digitalRead(PIN_FAN_HALL) == state && CurrentTime - StartTime < 1000000) { //hold while low
                                                 // hold here 1s while it doesnt change state/rev
        CurrentTime = micros();                    //**keep updating start counter
        OldState  = state;                      //** keep last state updated
    }
    if (debug) Serial.print("init_step ");

    while (true) {
        OldState = digitalRead(PIN_FAN_HALL);
        while (digitalRead(PIN_FAN_HALL) == OldState) {
            CurrentTime = micros(); //re-read current time so we get a more accurate sampling (avoiding slow serial writes)
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

void setup() {
    if (debug) Serial.begin(57600);
    
    //set all pins
    if (debug) Serial.println("setting up pin modes");

    //TODO: use ports registers
    pinMode(PIN_FAN_HALL, PIN_MODE_FAN_HALL);                                             //fan speed pin setup
    pinMode(PIN_IR_READ, PIN_MODE_IR_READ);                                               //infrared pin setup
    pinMode(PIN_ANALOG_TSENSE_INPUT_CASE, PIN_MODE_ANALOG_TSENSE_INPUT_CASE);             //analog case tsense pin setup
    pinMode(PIN_ANALOG_TSENSE_INPUT_HEXCHANGER, PIN_MODE_ANALOG_TSENSE_INPUT_HEXCHANGER); //analog HEx tsense pin setup
    pinMode(PIN_TSENSE_EMULATOR_CASE, PIN_MODE_TSENSE_EMULATOR);                          // tsense emulator case pin setup
    pinMode(PIN_TSENSE_EMULATOR_HEXCHANGER, PIN_MODE_TSENSE_EMULATOR);                    // tsense emulator hex pin setup
    
    //ACS712 init
    
    sensor.calibrate();

    //Let AC mcu control flap
    pinMode(PIN_STEPPER_COIL_A, INPUT);    
    pinMode(PIN_STEPPER_COIL_A_bar, INPUT);
    pinMode(PIN_STEPPER_COIL_B, INPUT);    
    pinMode(PIN_STEPPER_COIL_B_bar, INPUT);

    if (debug) Serial.println("setup() done.");
}

void loop() {
    //1 - Init config/settings
    //2 - Read temps
    //3 - Write analog tsense emulator
    //4 - wait for AC mainboard to move stepper to startup position (could be moved after we read everything on 5)
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


    if (debug) Serial.print("loop() read_fan_speed(): \n");
    read_fan_speed();

    // if (debug) Serial.print("loop() calculate_pwm_duty(): \n");
    // calculate_pwm_duty();

    if (debug) Serial.print("loop() read_analog_tsenses(): \n");
    read_analog_tsenses();

    if (debug) Serial.print("loop() read_db18s20(): \n");
    read_db18s20();

    if (debug) Serial.print("loop() read_acs712(): \n");
    read_acs712();

    // if (debug) Serial.print("loop() stepper_flap(): \n");
    // stepper_flap();
}