/*  Drives LED array on wall  */

const int dataPin = 2;  // connected to DIN (1) of MAX7219 (yellow)
const int latchPin = 5;  // connected to LOAD (12) of MAX7219 (green)
const int clockPin = 7; // connected to CLK (13) of MAX7219 (white)
const int switchPin = A0;
const int whitebuttonPin = A2;
const int bluebuttonPin = A1;
//const int codes[21] = {265, 521, 772, 1028, 1298, 1554, -1, 274, 530, 777, 1033, 1284, 1540, -1, 260, 516, 786, 1042, 1289, 1545, -1}; // diagonal stripes
//const int codes[21] = {260, 513, 785, 1042, 1290, 1548, -1, 266, 524, 772, 1025, 1297, 1554, -1, 273, 530, 778, 1036, 1284, 1537, -1}; // forward arrows
const int codes[70] = {257, 512, 768, 1024, 1280, 1536, -1, 258, -1, 260, -1, 264, -1, 272, -1, 256, 513, -1, 514, -1, 516, -1, 520, -1, 528, -1, 512, 769, -1, 770, -1, 772, -1, 776, -1, 784, -1, 768, 1025, -1, 1026, -1, 1028, -1, 1032, -1, 1040, -1, 1024, 1281, -1, 1282, -1, 1284, -1, 1288, -1, 1296, -1, 1280, 1537, -1, 1538, -1, 1540, -1, 1544, -1, 1552, -1};
const int codelen = 70;
int p = 0;
const int ndigits = 6;
//byte leds[ndigits] = {0, 0, 0, 0, 0, 0};
//byte modified[ndigits] = {1, 1, 1, 1, 1, 1};

// shifts a 16-bit message into the MAX7219 input register, then latches it.
void sendData(int data) {
  digitalWrite(clockPin, LOW);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, data >> 8);
  shiftOut(dataPin, clockPin, MSBFIRST, data);
  digitalWrite(latchPin, HIGH);
}

void allOff() {
  int offsignals[6] = {256, 512, 768, 1024, 1280, 1536};
  for (int i = 0; i < 6; i++)
    sendData(offsignals[i]);
}

// turns all leds on, then off using test mode
void testFlash(int ms) {
  sendData(0x0F01); // enable test mode
  delay(ms);
  sendData(0x0F00); // disable test mode
  delay(ms);
}

// updates LEDS on each digit that has changed
//void updateLEDs() {
//  for (int i = 0; i < ndigits; i++) {
//    if (modified[i]) {         // check if changed
//      sendData(((i+1) << 8) + leds[i]);  // send XXXX digt ledh ledl
//      modified[i] = 0;         // set as unchanged
//    }
//  }
//}

//void four_step() {
//  leds[0] = 64; // turn on 0A
//  modified[0] = 1;
//  updateLEDs();
//  delay(500);
//
//  leds[0] = 32; // turn off 0A, turn on 0B,
//  modified[0] = 1;
//  updateLEDs();
//  delay(500);
//
//  leds[0] = 0;  // turn off 0
//  modified[0] = 1;
//  leds[1] = 64; // turn on 1A
//  modified[1] = 1;
//  updateLEDs();
//  delay(500);
//
//  leds[1] = 32; // turn off 1A, turn on 1B
//  modified[1] = 1;
//  updateLEDs();
//  delay(500);
//  leds[1] = 0;  // turn off 1
//  modified[1] = 1;
//}

void setup() {
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(switchPin, INPUT);
  pinMode(whitebuttonPin, INPUT);
  pinMode(bluebuttonPin, INPUT);
  sendData(0x0900);       // set all digits to no-decode mode
  sendData(0x0A07);       // set intensity to 50%
  sendData(0x0B00 + ndigits - 1);  // set scan range to ndigits
  testFlash(500);
  testFlash(500);
  testFlash(500);
  allOff();
  sendData(0x0C01);       // disable shutdown mode to begin normal operation
}


void loop() {
//    while(codes[p] != -1) {
//      sendData(codes[p]);
//      p = (p + 1) % codelen;
//    }
//    p = (p + 1) % codelen;
//    delay(250);
  if (digitalRead(bluebuttonPin) || digitalRead(whitebuttonPin) || digitalRead(switchPin))
    sendData(0x0F01); // enable test mode
  else
    sendData(0x0F00); // disable test mode
//  testFlash(500);
  delay(30);
}
