
// USAGE:
// The goal of this program is to verify that your hall sensors
// are connected correctly, as well as positioned correctly.
// spinning clockwise, you should see the following sequence of bits
// being output to the serial monitor: 101 001 011 010 110 100
// each of the 3 bits represents a hall sensor, 1 is high,0 is low.
// Note: in the full cube program, the hall sensors are on interrupts rather than a busy loop.
//TROUBLESHOOTING:
// If you do not see the correct output,your hall sensors are likely
// positioned or wired incorrectly. This can be corrected using an oscilloscope
// as shown in the project repository on GitHub.

// If 1, states will be output numerically, ie. 3 instead of 011
#define NUMERICAL_STATES 0

//define hall Sensor pins
const short HA = 2;
const short HB = 3;
const short HC = 4;

short state = -1;
void setup() {
  Serial.begin(115200);

  // The A1304 output pin must be pulled up to 3.3v
  pinMode(HA, INPUT_PULLUP);
  pinMode(HB, INPUT_PULLUP);
  pinMode(HC, INPUT_PULLUP);
  pinMode(13, OUTPUT);

  delay(1000);
  // turn on the on-board LED to indicate logging has started.
  digitalWrite(13, HIGH);
}

// Get the current state of the hall sensors.
// if the state has changed we must commutate windings.
void loop() {
  int new_state = GetHallState();
  if (state != new_state) {
    if (NUMERICAL_STATES)
      Serial.println(new_state);
    state = new_state;
  }
}

// Returns the current position of the rotor.
// -1 should not be possible, if hall sensors don't match the 6 steps E-Stop.
short GetHallState() {
  int hallA = digitalRead(HA);
  int hallB = digitalRead(HB);
  int hallC = digitalRead(HC);

  if (!NUMERICAL_STATES) {
    Serial.print(hallA);
    Serial.print(hallB);
    Serial.println(hallC);
  }
  // Converts hall sensor readings to numerical states
  // 101 001 011 010 110 100
  if (hallA && !hallB && hallC)
    return 1;
  else if (!hallA && !hallB && hallC)
    return 2;
  else if (!hallA && hallB && hallC)
    return 3;
  else if (!hallA && hallB && !hallC)
    return 4;
  else if (hallA && hallB && !hallC)
    return 5;
  else if (hallA &&  !hallB && !hallC)
    return 6;
  else {
    // This should never happen, if it somehow does, states of the hall sensors are
    // printed at the time of failure for debugging purposes. This also calls
    // the emergency stop in the full program.
    Serial.print(hallA);
    Serial.print(hallB);
    Serial.println(hallC);
  }
}
