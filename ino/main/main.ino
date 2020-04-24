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
const int OFFSIGNALS[6] = {256, 512, 768, 1024, 1280, 1536}; // MAX7219 messages to turn off each column
const int DECAYPROFILESUSTAIN[5] = {2000, 3000, 3700, 5000, 8000}; // decay times for a sustained note
const int DECAYPROFILEFALL[5] = {40, 80, 120, 160, 200}; // decay times for a falling note
enum globalmodes{PIANO, AUTO};      // global switch modes
enum globalmodes globalmode;        // global switch mode variable
enum pianomodes{STARRY, COLUMNS};   // global pianomodes
enum pianomodes pianomode;          // global pianomode variable
enum automodes{RAINY, ALLON};       // global automodes
enum automodes automode;            // global automode variable

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
unsigned long scheduled[30] = {0};  // holds timestamps for when scheduled notes should come on - also used by rainy
uint8_t maxedout[3] = {0};          // flag for when each third of assigned array is totally full

// Column structs
unsigned long coldecays[30] = {0};    // keeps track of time at which each column segment will decay !! I should just reuse scheduled[] for this
uint8_t colstatus[6] = {0};           // in COLUMN mode: 0 means released (totally off or decaying in FALL pattern), 1 means on under key press or press + sustain, 2 means on under sustain alone (1 and 2 are decaying in SUSTAIN pattern)
enum coldecaystyles{SUSTAIN, FALL};   // column decay styles for COLUMNS mode

// Rainy structs
unsigned long nextdroptime = 0;        // time to launch the next raindrop
// reuses scheduled[]                  // times to illuminate next falling drops
unsigned long scheduledoff[30] = {0};  // times to deluminate falling drops
struct raindrop { 
  int col;
  int speed;
  int length;
  int depth;
  unsigned long updatetime;
};
struct raindrop raindrops[9];
int nextdropid = 0; // pointer through raindrops



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
  for (int i = 0; i < 6; i++)
    sendData(OFFSIGNALS[i]);
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

// when exiting starry mode, zeroes at starry mode data structures to return all data structures to original condition
void resetStarryMode() {
  memset(status, 0, sizeof(status));
  memset(assigned, 0, sizeof(assigned));
  memset(timestamps, 0, sizeof(timestamps));
  memset(scheduled, 0, sizeof(scheduled));
  memset(maxedout, 0, sizeof(maxedout));
  allOff();
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
  if (pos == -1) // all lights on, just evict randomly
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

void processStarryTimeDelays(unsigned long now) {
  for (int i = 0; i < 30; i++) {
    if (status[i] == 3 && now >= scheduled[i]) {
      illuminate(i);
      timestamps[i] = now;
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

// when exiting column mode, zeroes out column data structures to leave all data structures in original condition, and turns off all lights
void resetColMode() {
  memset(status, 0, sizeof(status));
  memset(colstatus, 0, sizeof(colstatus));
  memset(coldecays, 0, sizeof(coldecays));
  allOff();
}

// returns the index (0-5) of a column to use in column mode
// columns are not tied to particular notes, but rather note ranges. Every time a new note is played in the range, it re-fires the column.
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
  // Serial.print("setting decay times relative to time "); Serial.println(now);
  for (int i = 0; i < 5; i++) { // reset and overwrite column decays in one pass. i counts from bottom of column to top
    if (i < height) {
      if (style == SUSTAIN)
        coldecays[col * 5 + 4 - i] = now + DECAYPROFILESUSTAIN[height - 1 - i];
      else
        coldecays[col * 5 + 4 - i] = now + DECAYPROFILEFALL[height - 1 - i];
    } else {
      coldecays[col * 5 + 4 - i] = 0;
    }
    // Serial.print("level "); Serial.print(i); Serial.print(" time "); Serial.println(coldecays[col * 5 + 4 - i]);
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

void processColTimeDelays(unsigned long now) {
  for (int i = 0; i < 30; i++) {
    if (coldecays[i] != 0 && now >= coldecays[i]) {
      deluminate(i, false);
      coldecays[i] = 0;
    }
  }
}

// #####################
// ALL ON MODE FUNCTIONS
// #####################

void startAllOn() {
  sendData(0x0F01); // enable test mode
}

void resetAllOnMode() {
  sendData(0x0F00); // disable test mode
}

// ####################
// RAINY MODE FUNCTIONS
// ####################

// on random time intervals, trigger "droplets" to fall down a column.
// droplets have a random length (1-3) and speed
// dropletes schedule themselves one jump at a time, so on each illumination, it schedules the next one. Check if there is a pending schedule there, and only replace it if the new one is earlier
// On each delumination, schedule the next one. Check if there is a pending delumination scheduled there, and only overwrite if the new one is later

// returns a random future time between 100 and 2600 ms from now, quadratically weighted towards 100
unsigned long getNextDropTime() {
  return millis() + 100 + sq(rand() % 50);
}

// schedules next illumination and delumination for a particular drop, respecting pre-existing schedules, and updates drop struct
void scheduleNext(unsigned long now, int dropid) {
  if (raindrops[dropid].depth < 5) { // schedule a new illumination and delumination
    if (dropid == 4) {
      Serial.print("scheduling drop 4, depth: "); Serial.print(raindrops[dropid].depth); Serial.print(" time: "); Serial.println(raindrops[dropid].updatetime);
    }
    int pos = raindrops[dropid].col * 5 + raindrops[dropid].depth;
    unsigned long newschedule = raindrops[dropid].updatetime + raindrops[dropid].speed;
    if (scheduled[pos] < now || scheduled[pos] > newschedule) // only overwrite if empty/old existing, or existing is later than new
      scheduled[pos] = newschedule;
    unsigned long newscheduleoff = newschedule + (raindrops[dropid].speed * raindrops[dropid].length);
    if (scheduledoff[pos] < now || scheduledoff[pos] < newscheduleoff) // only overwrite if empty/old existing, or existing is sooner than new
      scheduledoff[pos] = newscheduleoff;
    raindrops[dropid].updatetime += raindrops[dropid].speed;
  } else {
    raindrops[dropid].updatetime = 0;
  }
  raindrops[dropid].depth++;
}


// void scheduleDrop(unsigned long now, int col, int speed, int length) {
//   for (int i = 1; i < 5; i++) {
//     scheduled   [col * 5 + i] =  now + (speed * i);
//     scheduledoff[col * 5 + i] =  now + (speed * (i + length));
//     scheduledoff[col * 5]     =  now + (speed * length);
//   }
//   // !! this will make new drops annihilate drops below them. One way to avoid this is to save drop state and have each drop schedule its own path downwards
// }

void launchDrop(unsigned long now, int dropid) {
  raindrops[dropid].col = rand() % 6;
  raindrops[dropid].speed = 120 + (rand() % 90);  // 120 to 210 ms per drop
  raindrops[dropid].length = (rand() % 100) < -1 ? 3 : 1 + (rand() % 2); // 10% length 3, 45% length 1, 45% length 2
  raindrops[dropid].depth = 0;
  raindrops[dropid].updatetime = now;
  scheduleNext(now, dropid);
  // illuminate(raindrops[dropid].col * 5); // illuminate top of column
}

void processRainyTimeDelays(unsigned long now) {
  // occasionally launch another drop
  if (now >= nextdroptime) {
    launchDrop(now, nextdropid);
    nextdroptime = getNextDropTime();
    nextdropid = (nextdropid + 1) % 9;
  }
  // check raindrop structs to make new schedules
  for (int i = 0; i < 9; i++)
    if (raindrops[i].updatetime != 0 && raindrops[i].updatetime <= now)
      scheduleNext(now, i);
  // check LED schedules and illuminate or deluminate
  for (int i = 0; i < 30; i++) {
    if (scheduled[i] != 0 && status[i] == 0 && now >= scheduled[i]) {
      illuminate(i);
      scheduled[i] = 0;
    } else if (scheduledoff[i] != 0 && status[i] == 1 && now >= scheduledoff[i]) {
      deluminate(i, false);
      scheduledoff[i] = 0;
    }
  }
}

void startRainy() {
  nextdroptime = getNextDropTime();
}

void resetRainyMode() {
  memset(status, 0, sizeof(status));
  memset(nextdroptime, 0, sizeof(nextdroptime));
  memset(nextdropid, 0, sizeof(nextdropid));
  memset(scheduled, 0, sizeof(scheduled));
  memset(scheduledoff, 0, sizeof(scheduledoff));
  memset(raindrops, 0, sizeof(raindrops));
  allOff();
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
  if (pianomode == STARRY) {
    processStarryNoteOn(noteid);
  } else if (pianomode == COLUMNS) {
    fireCol(findCol(noteid), vel);
  }
}

void processNoteOff(int noteid) {
  if (pianomode == STARRY) {
    processStarryNoteOff(noteid);
  } else if (pianomode == COLUMNS) {
    processColNoteOff(findCol(noteid));
  }
}

void processSustainOn(void) {
  sustain = 1;
}

void processSustainOff(void) {
  sustain = 0;
  if (pianomode == STARRY) {
    processStarrySustainOff();
  } else if (pianomode == COLUMNS) {
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
  if (globalmode == PIANO) {
    if (pianomode == STARRY) {
      resetStarryMode();
      pianomode = COLUMNS;
      Serial.println("changing pianomode to COLUMNS");
    } else if (pianomode == COLUMNS) {
      resetColMode();
      pianomode = STARRY;
      Serial.println("changing pianomode to STARRY");
    }
  } else if (globalmode == AUTO) {
    if (automode == ALLON) {
      resetAllOnMode();
      automode = RAINY;
      startRainy();
      Serial.println("changing automode to RAINY");
    } else if (automode == RAINY) {
      resetRainyMode();
      automode = ALLON;
      startAllOn();
      Serial.println("changing automode to ALLON");
    }
  }
}

// cycle the other direction
void changemodedown() {
  changemodeup();
}

// toggling switch from down to up (into auto mode)
void switchUp() {
  if (globalmode == PIANO) {
    if (pianomode == STARRY) {
      resetStarryMode();
    } else if (pianomode == COLUMNS) {
      resetColMode();
    }
    globalmode = AUTO;
    if (automode == ALLON) {
      startAllOn();
    } else if (automode == RAINY) {
      startRainy();
    }
  } else {
    Serial.println("Switch going up but already in auto mode. Should not get here");
  }
}

// toggling switch from up to down (into piano mode)
void switchDown() {
  if (globalmode == AUTO) {
    if (automode == RAINY) {
      resetRainyMode();
    } else if (automode == ALLON) {
      resetAllOnMode();
    }
    globalmode = PIANO;
    // !! I don't think I need to init any piano-mode specific stuff, but if I did, this is where I would do it.
  } else {
    Serial.println("Switch going down but already in piano mode. Should not get here");
  }
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
      if (globalmode == PIANO) // !! checking down here in case we need to keep polling midi buffer to flush it. But there could be side effects, like what if the piano must be turned on for auto to work? Not desirable. Also auto performance considerations of needlessly calling Usb.Task() and Midi.Recvdata(), although performance is fine for piano mode
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
  if (globalmode == PIANO) {
    if (pianomode == STARRY) 
      processStarryTimeDelays(now);
    else if (pianomode == COLUMNS) 
      processColTimeDelays(now);
  } else if (globalmode == AUTO) {
    if (automode == RAINY)
      processRainyTimeDelays(now);
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
        changemodedown();
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
        switchUp();
      else
        switchDown();
    }
  }
  lastSwitchState = reading;
}


// poll buttons and switch and respond
void processButtons() {
  pollSwitch();
  pollBlueButton();
  pollWhiteButton();
}

void initPins() {
  pinMode(LATCHPIN, OUTPUT);
  pinMode(CLOCKPIN, OUTPUT);
  pinMode(DATAPIN, OUTPUT);
  pinMode(SWITCHPIN, INPUT);
  pinMode(WHITEBUTTONPIN, INPUT);
  pinMode(BLUEBUTTONPIN, INPUT);
}

void initLEDS() {
  sendData(0x0900);  // set all digits to no-decode mode
  sendData(0x0A0F);  // set intensity to 100%
  sendData(0x0B05);  // set scan range to six digits
  allOff();
  testFlash(150);
  testFlash(150);
  testFlash(150);
  sendData(0x0C01);       // disable shutdown mode to begin normal operation
}

void initUSB() {
  if (Usb.Init() == -1) {
    Serial.println("USB failed to init");
    while (1); // halt
  }
  Serial.println("USB initialized successfully!");
}

// one-time setup on power-up
void setup() {
  Serial.begin(57600);
  initPins();
  initLEDS();
  initUSB();
  pollSwitch();
  randomSeed(analogRead(4));
  pianomode = STARRY;
  automode = RAINY;
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


// !! if compiled size starts to become a problem, one small tip is to use uint8_t or int8_t instead of int where possible

// todo:
// tune columns intensities (especially soft ones) to match actual intensity. Softest threshold is too soft
// tune colunmns horizontal interval widths to match common playing intervals, kind of like thirds in starry mode
// make columns re-flash repeated sustained columns like starry mode. Maybe also reflash decaying notes that are still on (I will get that for free I think)
// lucinda says sustained columns should decay faster so you can actually see them, and also maybe fire up as well

