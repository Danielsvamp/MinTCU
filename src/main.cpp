#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(53);

struct can_frame canMsg;

// SHIFT PINS
const int shift23 = 22;
const int shift34 = 23;
const int shift12_45 = 24;

// PWM PINS
const int mpc = 11;
const int spc = 12;
const int tcc = 13;

uint16_t accPedalPosition = 0;

enum Shifter {
  P = 0x08,
  R = 0x07,
  N = 0x06,
  D = 0x05,
  UP = 0x09,
  DN = 0x0A,
};

int currentGear = 2;
int newGear = -1;
bool shiftActive = false;
unsigned long shiftStart = 0;
unsigned long shiftDuration = 500;

struct shiftCalibration {
  int fromGear;
  int toGear;
  int solenoidPin;
  unsigned long duration;
};

Shifter shifterPosition = Shifter::P;

shiftCalibration shifts[] = {
  {1, 2, shift12_45, 500},
  {2, 1, shift12_45, 500},
  {2, 3, shift23,    500},
  {3, 2, shift23,    500},
  {3, 4, shift34,    500},
  {4, 3, shift34,    500},
  {4, 5, shift12_45, 500},
  {5, 4, shift12_45, 500},
};

void setup() {
  Serial.begin(115200);

  pinMode(shift23, OUTPUT);
  pinMode(shift34, OUTPUT);
  pinMode(shift12_45, OUTPUT);

  digitalWrite(shift12_45, LOW);
  digitalWrite(shift23, LOW);
  digitalWrite(shift34, LOW);

  pinMode(mpc, OUTPUT);
  pinMode(spc, OUTPUT);
  pinMode(tcc, OUTPUT);

  analogWrite(mpc, 0);
  analogWrite(spc, 0);
  analogWrite(tcc, 0);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("ready");
}

void startShift(int fromGear, int toGear) {
  Serial.println("startShift");
  for (auto &s : shifts) {
    if (s.fromGear == fromGear && s.toGear == toGear) {
      shiftStart = millis();
      digitalWrite(s.solenoidPin, HIGH);
      shiftDuration = s.duration;
      Serial.print("Solenoid HIGH: ");
      Serial.println(s.solenoidPin);
      return;
    }
  }
  Serial.println("No valid shift profile found!");
}

void requestShift(int dir) {
  if (shiftActive) return;

  newGear = currentGear + dir;

  if (newGear < 1 || newGear > 5) return;

  shiftActive = true;
  startShift(currentGear, newGear);

  Serial.print("Shifting to ");
  Serial.println(newGear);
}

void handleShifterMessage(uint8_t msg) {
  if (msg == Shifter::P || msg == Shifter::R || msg == Shifter::N) {
    shifterPosition = static_cast<Shifter>(msg);
  }
  else if (msg == Shifter::D) {
    shifterPosition = Shifter::D;
  }
  else if (msg == Shifter::UP && shifterPosition == Shifter::D && !shiftActive && currentGear < 5) {
    requestShift(+1);
    Serial.println("Requested upshift");
  }
  else if (msg == Shifter::DN && shifterPosition == Shifter::D && !shiftActive && currentGear > 1) {
    requestShift(-1);
    Serial.println("Requested downshift");
  }

  if ((msg & 0xF0) == 0x80) {
    Serial.print("Message: 0x");
    Serial.println(msg, HEX);
  }
}

void loop() {
  if (shiftActive && millis() - shiftStart >= shiftDuration) {
    digitalWrite(shift12_45, LOW);
    digitalWrite(shift23, LOW);
    digitalWrite(shift34, LOW);
    Serial.println("pins low");

    currentGear = newGear;
    shiftActive = false;
    Serial.println("Shift completed");
  }

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.can_id == 0x230) {
      uint8_t msg = canMsg.data[0];
      handleShifterMessage(msg);
    }

    if (canMsg.can_id == 0x212) {
      uint16_t accPedalPosition = (canMsg.data[2] << 8) | canMsg.data[3];
      Serial.print("Pedal Raw: ");
      Serial.print(accPedalPosition);
      Serial.print(" (");
      Serial.print((accPedalPosition * 100.0) / 65535.0);
      Serial.println("%)");
    }
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'u' && currentGear < 5) {
      requestShift(+1);
      Serial.println("Requested upshift");
    }
    else if (c == 'd' && currentGear > 1) {
      requestShift(-1);
      Serial.println("Requested downshift");
    }
    else if (c == '0') analogWrite(tcc, 0);
    else if (c == '1') analogWrite(tcc, 64);
    else if (c == '2') analogWrite(tcc, 128);
    else if (c == '3') analogWrite(tcc, 98/100*255);
  }
}

