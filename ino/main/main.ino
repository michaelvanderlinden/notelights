#include <usbh_midi.h>
#include <usbhub.h>
// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
USBH_MIDI Midi(&Usb);

const int DATAPIN = 2;  // connected to DIN (1) of MAX7219 (yellow)
const int LATCHPIN = 5;  // connected to LOAD (12) of MAX7219 (green)
const int CLOCKPIN = 7; // connected to CLK (13) of MAX7219 (white)
const int SWITCHPIN = A0;
const int WHITEBUTTONPIN = A2;
const int BLUEBUTTONPIN = A1;
const int DEBOUNCEDELAY = 30; // milliseconds to wait before registering a second press or release of a button or switch
const unsigned int RESCHEDULEDELAY = 100; // time delay in milliseconds to darken repeated sustained notes
const uint8_t MIDIBOUND1 = 53;   // notes below the F below middle C are in lower third (left two columns)
const uint8_t MIDIBOUND2 = 72;   // notes above the C above middle C are in upper third (right two columns)
const int DECAYPROFILESUSTAIN[5] = {2000, 3000, 3700, 5000, 8000}; // decay times for a sustained note
const int DECAYPROFILEFALL[5] = {40, 80, 120, 160, 200}; // decay times for a falling note
enum modes{STARRY, COLUMNS};   // global modes
enum modes mode;               // global mode variable

// global flags and structs
// all flat 30-arrays are arranged top to bottom, left to right. G on top (LSB), C on bottom (MSB)
uint8_t switchState = 0;       // on or off
uint8_t blueState = 0;          
uint8_t whiteState = 0;
uint8_t lastSwitchState = 0;
uint8_t lastBlueState = 0;          
uint8_t lastWhiteState = 0;
unsigned long lastBlueDebounceTime;
unsigned long lastWhiteDebounceTime;
unsigned long lastSwitchDebounceTime;
uint8_t sustain = 0;        // is sustain pedal down
uint8_t midibuf[3];         // stores midi input, up to 3 bytes
uint8_t status[30] = {0};
// STARRY mode: 0 means off, 1 means on under key press or press + sustain, 2 means on under sustain alone, 
// 3 means temporarily off under sustained re-press.
// Any note in state 3 is guaranteed to be both actively pressed and sustained. 
// COLUMN mode (and perhaps others) uses states 0 and 1

// Starry structs
uint8_t assigned[30] = {0}; // assigned persistent note ids of each light. Although the midi scale goes down to noteid 0, lowest note on 88-key piano (A0) has code 21
unsigned long timestamps[30] = {0}; // holds the timestamps for when notes were assigned
unsigned long scheduled[30] = {0};  // holds timestamps for when scheduled notes should come on
uint8_t maxedout[3] = {0};          // flag for when each third of assigned array is totally full

// Column structs
int coldecays[30] = {0};    // keeps track of time at which each column segment will decay
uint8_t colstatus[6] = {0}; // in COLUMN mode: 0 means released (totally off or decaying in FALL pattern), 1 means on under key press or press + sustain, 2 means on under sustain alone (1 and 2 are decaying in SUSTAIN pattern)
enum coldecaystyles{SUSTAIN, FALL};  // column decay styles for COLUMNS mode


// #################
// MAX7219 FUNCTIONS
// #################

// returns a MAX7219 update digit (column) message, based on the current state of status array
int buildMessage(int digit) {
  int start = digit * 5; // starting point for reading through status array
  int segments = 0;
  for (int i = 0; i < 5; i++)
    if (status[start + i] == 1 || status[start + i] == 2)
      segments += (1 << i); // segs indexed top-to-bottom with G on top as LSB
  return ((digit + 1) << 8) + segments; // digits indexed from 1
}

// shifts a 16-bit message into the MAX7219 input register, then latches it.
void sendData(int data) {
  digitalWrite(CLOCKPIN, LOW);
  digitalWrite(LATCHPIN, LOW);
  shiftOut(DATAPIN, CLOCKPIN, MSBFIRST, data >> 8);
  shiftOut(DATAPIN, CLOCKPIN, MSBFIRST, data);
  digitalWrite(LATCHPIN, HIGH);
}

// turns all leds off
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


// ################
// STARRY FUNCTIONS
// ################

// zeroes at starry mode data structures to return all data structures to original condition
void resetStarryMode() {
  memset(status, 0, sizeof(status));
  memset(assigned, 0, sizeof(assigned));
  memset(timestamps, 0, sizeof(timestamps));
  memset(scheduled, 0, sizeof(scheduled));
  memset(maxedout, 0, sizeof(maxedout));
}

// returns first index (0-29) in assigned array where id matches. If note id not found, return -1
int findNote(int noteid) {
  for (int i = 0; i < 30; i++)
    if (assigned[i] == noteid)
      return i;
  return -1;
}

// returns the offset / 10 (0, 1, or 2) by which to restrict note position, based on id and midi bounds
int getRangeOffset(int noteid) {
  if (noteid < MIDIBOUND1)
    return 0;
  if (noteid > MIDIBOUND2)
    return 2;
  return 1;
}

// returns a random unassigned spot in the proper third of assigned array. If maxedout, return -1. If no slot is found in 50 tries, set maxedout and return -1
int findOpenSlot(int noteid) {
  int offset = getRangeOffset(noteid);
  if (maxedout[offset])
    return -1;
  int attempt;
  for (int i = 0; i < 60; i++) {
    attempt = (rand() % 10) + (offset * 10);
    if (assigned[attempt] == 0)
      return attempt;
  }
  maxedout[offset] = 1;
  return -1;
}

// replaces oldest assigned note that is currently dark ~~or sustained~~ with noteid, and returns its position
int replaceOldest(int noteid) {
  int offset = getRangeOffset(noteid) * 10;
  int pos = -1;
  unsigned long oldestts = millis() + 1;
  for (int i = offset; i < offset + 10; i++) {
    if ((status[i] == 0 || status[i] == 2) && timestamps[i] < oldestts) {
      oldestts = timestamps[i];
      pos = i;
    }
  }
  if (pos == -1) // all lights on, just evict randomly // !! I think timestamp eviction is failing to evict oldest because we're always getting down here. This should be rare case
    pos = (rand() % 10) + offset;
  assigned[pos] = noteid;
  return pos;
}

void processStarryNoteOn(int noteid) {
  int pos = findNote(noteid);
  if (pos == -1) { // a. that note is not currently assigned
    pos = findOpenSlot(noteid);
    if (pos != -1) { // i. there is an unassigned space available - take it. I will assume for now that unassigned spaces will have status 0
      assigned[pos] = noteid;
      illuminate(pos);
    } else { // ii. there is no such space available, so unassign the oldest currently assigned space (that's not in state 1 or 3) and take it
      pos = replaceOldest(noteid);
      if (status[pos] == 2)
        deluminate(pos, true); // bounce sustained evicted notes
      else
        illuminate(pos);
    }
  } else { // b. that note is currently assigned
    if (status[pos] == 0) { // i. its spot is dark (s0) - illuminate
      illuminate(pos);
    } else if (status[pos] == 2) { // ii. its spot is sustained (s2) - turn it off and schedule it to be re-lit 
      deluminate(pos, true);
    }
    // if spot is scheduled (s3), leave pending schedule alone. If somehow already in s1, do nothing
  }
  timestamps[pos] = millis(); // record time of playing
}

void processStarryNoteOff(int noteid) {
  int pos = findNote(noteid);
  if (pos != -1) {   // note should already be assigned, but just in case. --important and necessary check when switching modes
    if (status[pos] == 1) {
      if (sustain)
        status[pos] = 2;   // if illuminated (s1) and sustain pedal is on, set status to 2
      else
        deluminate(pos, false);  // if illuminated (s1) and sustain pedal is off, deluminate
    } else if (status[pos] == 3) {
      if (sustain) {
        illuminate(pos);   // if scheduled (s3) and sustain pedal is on, cancel schedule by immediately illuminating and then setting status to 2
        status[pos] = 2;
      } else {
        status[pos] = 0;   // if scheduled (s3) and sustain pedal is off, cancel schedule by immediately setting status to 0
      }
    }
  }
}

void processStarrySustainOff() {
  for (int i = 0; i < 30; i++) {
    if (status[i] == 2) {
      deluminate(i, false);
    } else if (status[i] == 3) {
      illuminate(i);   // all scheduled (s3) notes must immediately illuminate and move to status 1
    }
  }
}

// #####################
// COLUMN MODE FUNCTIONS
// #####################

// for columns: each note should instantiate a column using velocity.
// column matrix should have the 6 columns x up to 5 timestamps signaling when to drop down each level.
// note on should drive initial column activation and schedule a set of up to 5 decay timestamps
// note off should reschedule the 5 decay timestamps to a rapid decay

// zeroes out column data structures to leave all data structures in original condition
void resetColMode() {
  memset(status, 0, sizeof(status));
  memset(colstatus, 0, sizeof(colstatus));
  memset(coldecays, 0, sizeof(coldecays));
}

// returns the index (0-5) of a column to use in column mode
// columns are not tied to particular notes, but rather note ranges. Every time a new note is played in the range, it re-fires the column. !! this might looks ugly or just max-out the wall.
int findCol(int noteid) {
  int scaled = noteid - 21; // 88 keys. Ranges: 20 14 10 10 14 20
  if (scaled >= 68)
    return 5;
  if (scaled >= 54)
    return 4;
  if (scaled >= 44)
    return 3;
  if (scaled >= 34)
    return 2;
  if (scaled >= 20)
    return 1;
  return 0;
}

// takes a velocity and returns the height of a column (1-5) on initial fire
int setColHeight(int vel) {
  if (vel < 30)
    return 1;
  if (vel < 50)
    return 2;
  if (vel < 65)
    return 3;
  if (vel < 85)
    return 4;
  return 5;
}

// returns the current height of notes in column
int checkColHeight(int col) {
  int height = 0;
  for (int i = col*5; i < col*5 + 5; i++)
    if (status[i])
      height++;
  return height;
}

// sets the status lights of lights in a column to 1 or 0, depending on given height, and illuminates them.
void raiseCol(int col, int height) {
  for (int i = 0; i < 5; i++) { // reset and overwrite column in one pass. i counts from bottom of column to top
    if (i < height)
      status[col * 5 + 4 - i] = 1;
    else
      status[col * 5 + 4 - i] = 0;
  }
  sendData(buildMessage(col));
}

// resets and overwrites column decays starting from a particular height, matching decay style (SUSTAIN or FALL)
void setColDecays(int col, int height, enum coldecaystyles style) {
  unsigned long now = millis();
  for (int i = 0; i < 5; i++) { // reset and overwrite column decays in one pass. i counts from bottom of column to top
    if (i < height) {
      if (style == SUSTAIN)
        coldecays[col * 5 + 4 - i] = now + DECAYPROFILESUSTAIN[height - 1 - i];
      else
        coldecays[col * 5 + 4 - i] = now + DECAYPROFILEFALL[height - 1 - i];
    } else {
      coldecays[col * 5 + 4 - i] = 0;
    }
  }
}

// raises a column at the specified velocity and sets sustain decay pattern
void fireCol(int col, int vel) {
  int height = setColHeight(vel);
  raiseCol(col, height);
  setColDecays(col, height, SUSTAIN);
  colstatus[col] = 1;
}

// when the note sustaining a column is cut off, reschedule decay style to the FALL pattern. This depends on the current height of the column
void releaseCol(int col) {
  int height = checkColHeight(col);
  if (height > 0) {
    setColDecays(col, height, FALL);
    colstatus[col] = 0;
  }
}

void processColNoteOff(int col) {
  if (sustain) {
    colstatus[col] = 2;
  } else {
    releaseCol(col);
  }
}

void processColSustainOff() {
  for (int i = 0; i < 6; i++) {
    if (colstatus[i] == 2)
      releaseCol(i);
  }
}

// #######################
// MIDI RESPONSE FUNCTIONS
// #######################

// writes 1 to status[pos] and sends message to update led display
void illuminate(int pos) {
  status[pos] = 1;
  sendData(buildMessage(pos / 5));  
}

// writes 0 to status[pos] (or 3 if rescheduling) and sends message to update led display
// schedules a future illumination if directed
void deluminate(int pos, bool reschedule) {
  status[pos] = reschedule ? 3 : 0;
  if (reschedule)
    scheduled[pos] = millis() + RESCHEDULEDELAY;
  sendData(buildMessage(pos / 5));
}

void processNoteOn(uint8_t noteid, uint8_t vel) {
  if (mode == STARRY) {
    processStarryNoteOn(noteid);
  } else if (mode == COLUMNS) {
    fireCol(findCol(noteid), vel);
  }
}

void processNoteOff(int noteid) {
  if (mode == STARRY) {
    processStarryNoteOff(noteid);
  } else if (mode == COLUMNS) {
    processColNoteOff(findCol(noteid));
  }
}

void processSustainOn(void) {
  sustain = 1;
}

void processSustainOff(void) {
  sustain = 0;
  if (mode == STARRY) {
    processStarrySustainOff();
  } else if (mode == COLUMNS) {
    processColSustainOff();
  }
}

// Each note illuminates a random led for the duration of the note.
// Sustain pedal will keep notes illuminated until pedal is released.
void respondToMidiMessage() {
  if (midibuf[0] >> 4 == 0x9)
    processNoteOn(midibuf[1], midibuf[2]);
  else if (midibuf[0] >> 4 == 0x8)
    processNoteOff(midibuf[1]);
  else if (!sustain && midibuf[0] >> 4 == 0xB && midibuf[1] == 0x40 && midibuf[2] != 0)
    processSustainOn();
  else if (sustain && midibuf[0] >> 4 == 0xB && midibuf[1] == 0x40 && midibuf[2] == 0)
    processSustainOff();
}


// ##############
// MAIN FUNCTIONS
// ##############

// reset used data structures and cycle through list
void changemodeup() {
  Serial.println("changing mode");
  if (mode == STARRY) {
    resetStarryMode();
    mode = COLUMNS;
  }
  if (mode == COLUMNS) {
    resetColMode();
    mode = STARRY;
  }
}

// cycle the other direction
void changemodedown() {
  changemodeup();
}

// Poll USB MIDI Controller and respond to keyboard inputs
void processMidi(void) {
  Usb.Task();
  uint8_t bsize;     // size of midi input
  int dataexists = 0;
  do {
    if ((bsize = Midi.RecvData(midibuf)) > 0) {
      dataexists = 1;
//      for (int i = 0; i < bsize; i++) {
//        Serial.print(midibuf[i], HEX);
//        Serial.print(" ");
//      }
      respondToMidiMessage();
    }
  } while (bsize > 0);
//  if (dataexists) {
//    Serial.print("\n");
//  }
}

// take any led actions that are scheduled for some time in the future
void processTimeDelays() {
  unsigned long now = millis(); 
  if (mode == STARRY) {
    for (int i = 0; i < 30; i++) {
      if (status[i] == 3 && now >= scheduled[i]) {
        illuminate(i);
        timestamps[i] = now;
      }
    }
  } else if (mode == COLUMNS) {
    for (int i = 0; i < 30; i++) {
      if (coldecays[i] != 0 && now >= coldecays[i]) {
        deluminate(i, false);
        coldecays[i] = 0;
      }
    }
  }
}

void pollBlueButton() {
  int reading = digitalRead(BLUEBUTTONPIN);
  if (reading != lastBlueState) {
    lastBlueDebounceTime = millis();
  }
  if ((millis() - lastBlueDebounceTime) > DEBOUNCEDELAY) {
    if (reading != blueState) {
      blueState = reading;
      if (blueState == 1)
        changemodeup();
    }
  }
  lastBlueState = reading;
}

void pollWhiteButton() {
  int reading = digitalRead(WHITEBUTTONPIN);
  if (reading != lastWhiteState) {
    lastWhiteDebounceTime = millis();
  }
  if ((millis() - lastWhiteDebounceTime) > DEBOUNCEDELAY) {
    if (reading != whiteState) {
      whiteState = reading;
      if (whiteState == 1)
        changemodeup();
    }
  }
  lastWhiteState = reading;
}

void pollSwitch() {
  int reading = digitalRead(SWITCHPIN);
  if (reading != lastSwitchState) {
    lastSwitchDebounceTime = millis();
  }
  if ((millis() - lastSwitchDebounceTime) > DEBOUNCEDELAY) {
    if (reading != switchState) {
      switchState = reading;
      if (switchState == 1)
        sendData(0x0F01); // enable test mode
      else
        sendData(0x0F00); // disable test mode
    }
  }
  lastSwitchState = reading;
}

// poll buttons and switch and respond
void processButtons() {
  pollBlueButton();
  pollWhiteButton();
  pollSwitch();
}

// one-time setup on power-up
void setup() {
  Serial.begin(57600);
  pinMode(LATCHPIN, OUTPUT);
  pinMode(CLOCKPIN, OUTPUT);
  pinMode(DATAPIN, OUTPUT);
  pinMode(SWITCHPIN, INPUT);
  pinMode(WHITEBUTTONPIN, INPUT);
  pinMode(BLUEBUTTONPIN, INPUT);
  sendData(0x0900);  // set all digits to no-decode mode
  sendData(0x0A0F);  // set intensity to 100%
  sendData(0x0B05);  // set scan range to six digits
  allOff();
  testFlash(250);
  testFlash(250);
  testFlash(250);
  sendData(0x0C01);       // disable shutdown mode to begin normal operation

  if (Usb.Init() == -1) {
    Serial.println("USB failed to init");
    while (1); // halt
  }
  Serial.println("USB initialized successfully!");
  mode = STARRY;
  delay(50);
}

// main loop after setup
void loop() {
  processButtons();
  processMidi();
  processTimeDelays();

  delay(2); // !! rather than delay, maybe just use millis to poll the midi source every 2 or 3 ms.
}


// could adjust by range: high (melody) notes get solo treatment, while bass notes are bundled to avoid clutter


// sprint:
// -- create ids array separate from on-off array to store active notes
// -- make new notes pick an empty slot in ids array or displace an older one - also requires a timestamp array of note ages
// -- sustains: if a note is sustained and plays again, turn it off and schedule it to turn on 100ms later. We will need to carefully keep track of scheduled notes and make sure not to displace them in that interval
// make notes select a slot based on third

// !! at some point I should probably ENUM the statuses
// !! if compiled size starts to become a problem, one small tip is to use uint8_t or int8_t instead of int where possible
// !! be carefule with mode switching, since I'm reusing data structures like status[30]. Remember to zero out appropriate flags and data structures when opening a new mode
