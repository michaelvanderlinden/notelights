#include <usbh_midi.h>
#include <usbhub.h>
// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
USBH_MIDI Midi(&Usb);

// global consts and definitions
#define DATAPIN 2   // connected to DIN (1) of MAX7219 (yellow)
#define LATCHPIN 5  // connected to LOAD (12) of MAX7219 (green)
#define CLOCKPIN 7  // connected to CLK (13) of MAX7219 (white)
#define SWITCHPIN A0
#define WHITEBUTTONPIN A2
#define BLUEBUTTONPIN A1
#define DEBOUNCEDELAY 30    // milliseconds to wait before registering a second press or release of a button or switch
#define RESCHEDULEDELAY 100 // time delay in milliseconds to darken repeated sustained notes
#define MIDIBOUND1 53       // notes below the F below middle C are in lower third (left two columns)
#define MIDIBOUND2 72       // notes above the C above middle C are in upper third (right two columns)
#define TRAVELERBUFSIZE 12  // max number of travelers to hold in memory at once
static const int OFFSIGNALS[6] = {256, 512, 768, 1024, 1280, 1536}; // MAX7219 messages to turn off each column
static const int DECAYPROFILESUSTAIN[5] = {2000, 3000, 3700, 5000, 8000}; // decay times for a sustained note
static const int DECAYPROFILEFALL[5] = {40, 80, 120, 160, 200}; // decay times for a falling note

// global flags and structs
// all flat 30-arrays are arranged top to bottom, left to right. G on top (LSB), C on bottom (MSB)
enum globalmodes{PIANO, AUTO};         // global switch modes
enum globalmodes globalmode;           // global switch mode variable
enum pianomodes{STARRY, COLUMNS};      // global pianomodes
enum pianomodes pianomode;             // global pianomode variable
enum automodes{RAINY, SNOWY, SNAKEY, METEOR, ALLON};   // global automodes
enum automodes automode;               // global automode variable
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
// COLUMN mode and RAINY mode uses states 0 and 1 with illuminate and deluminate

// Starry structs
uint8_t assigned[30] = {0}; // assigned persistent note ids of each light. Although the midi scale goes down to noteid 0, lowest note on 88-key piano (A0) has code 21
unsigned long timestamps[30] = {0}; // holds the timestamps for when notes were assigned
unsigned long scheduled[30] = {0};  // holds timestamps for when scheduled notes should come on
uint8_t maxedout[3] = {0};          // flag for when each third of assigned array is totally full

// Column structs
// recycles scheduled[30] = {0};      // keeps track of time at which each column segment will decay
uint8_t colstatus[6] = {0};           // keeps track of status for each column. 0 means released (totally off or decaying in FALL pattern), 1 means on under key press or press + sustain, 2 means on under sustain alone (1 and 2 are decaying in SUSTAIN pattern)
enum coldecaystyles{SUSTAIN, FALL};   // column decay styles for COLUMNS mode

// Traveler structs for Rainy, Snowy, Snakey, Meteor modes
unsigned long nextlaunchtime = 0;       // time to launch the next traveler
struct traveler { 
  int8_t lane;
  int speed;
  int8_t length;
  int8_t progress; // progress of leading edge of traveler
  int8_t row; // leading edge, for meteors only
  int8_t col; // leading edge, for meteors only
  bool direction; // for snakes, 0 means left (or up), 1 means right (or down). For meteors, 0 means shallow angle, 1 means steep angle
  unsigned long nextupdate;
};
struct traveler travelers[TRAVELERBUFSIZE]; // keeps track of parameters and status of each active traveler
uint8_t nexttravelerid = 0; // index through travelers


// witchcraft that tells you how much RAM is free
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


// #################
// MAX7219 FUNCTIONS
// #################

// returns a 16-bit MAX7219 update digit (column) message, based on the current state of status array
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
  delay(1); // Sometimes lights got left on when I changed modes and I'm superstitious that it was because of too-fast latching
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
  memset(scheduled, 0, sizeof(scheduled));
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
        scheduled[col * 5 + 4 - i] = now + DECAYPROFILESUSTAIN[height - 1 - i];
      else
        scheduled[col * 5 + 4 - i] = now + DECAYPROFILEFALL[height - 1 - i];
    } else {
      scheduled[col * 5 + 4 - i] = 0;
    }
    // Serial.print("level "); Serial.print(i); Serial.print(" time "); Serial.println(scheduled[col * 5 + 4 - i]);
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
    if (scheduled[i] != 0 && now >= scheduled[i]) {
      deluminate(i, false);
      scheduled[i] = 0;
    }
  }
}

// #######################
// TRAVELER MODE FUNCTIONS
// #######################

// This function applies to RAINY, SNOWY, SNAKEY, and METEOR modes
// They all involve launching a single light or string of lights to traverse the array vertically or horizontally

void resetTravelerMode() {
  memset(status, 0, sizeof(status));
  memset(nextlaunchtime, 0, sizeof(nextlaunchtime));
  memset(nexttravelerid, 0, sizeof(nexttravelerid));
  memset(travelers, 0, sizeof(travelers));
  allOff();
}

// This  function applies to RAINY, SNOWY, and METEOR modes
// RAINY: returns a random future time between 80 and 2000 ms from now, quadratically weighted towards 100
// SNOWY: time is between 300 and 450 ms from now
// METEOR: time is time is between 0.5s and 7s from now.
unsigned long getNextLaunchTime(unsigned long now) {
  if (automode == RAINY)
    return now + 80 + sq(rand() % 45);
  else if (automode == SNOWY)
    return now + 300 + (rand() % 150);
  else if (automode == METEOR)
    return now + 500 + (rand() % 6500);
}

// ##############################
// RAINY AND SNOWY MODE FUNCTIONS
// ##############################

// on random time intervals, trigger "droplets" to fall down a column.
// Rain droplets have a random length (1-3) and speed. Snow will have length 1 and slower speed, but more drops launched per second
// droplets advance themselves one row at a time. Each illumination proceeds unilaterally. Delumination proceeds if no other drop is holding onto the spot to be deluminated

void startDropMode() {
  for (int i = 0; i < TRAVELERBUFSIZE; i++)
    travelers[i].lane = -1; // mark all entries as invalid
  nextlaunchtime = getNextLaunchTime(millis());
}


// returns true if any drop is currently lighting up the led at col, row
bool anyDropsActive(int8_t col, int row) {
  for (int i = 0; i < TRAVELERBUFSIZE; i++) {
    if (col == travelers[i].lane && row <= travelers[i].progress && row > travelers[i].progress - travelers[i].length) // invalid (-1 lane) entries will not match col
      return true;
  }
  return false;
}

// advances a rain or snow drop one step down a column
void advanceDrop(struct traveler * drop) {
  drop->progress++;
  if (drop->progress <= 4) {
    status[drop->lane * 5 + drop->progress] = 1; // advance front of drop, but not farther than bottom
  }
  int taildepth = drop->progress - drop->length; // position of tail of drop that may be deluminated
  if (taildepth >= 0 && taildepth <= 4 && !anyDropsActive(drop->lane, taildepth)) // tail is on board and no other drops holding on to taildepth position
    status[drop->lane * 5 + taildepth] = 0; // release tail of drop
  // Serial.print("sending data for lane "); Serial.println(drop->lane);
  sendData(buildMessage(drop->lane));  // illuminate front and deluminate tail in one message
  drop->nextupdate += drop->speed; // schedule next update
  if (taildepth >= 4) // tail just dropped off board
    drop->lane = -1; // mark buffer entry as invalid
}

// launches a rain or snow drop from the top of a random column
void launchDrop(unsigned long now, struct traveler * drop) {
  drop->lane = rand() % 6; // column
  drop->speed = (automode == RAINY ? 100 + (rand() % 130) : 400 + (rand() % 150));  // RAINY: 100 to 230 ms per descent step; SNOWY: 400 to 550 ms per step
  drop->length = (automode == RAINY ? ((rand() % 100) < 10 ? 3 : 1 + (rand() % 2)) : 1); // RAINY: 10% length 3, 45% length 1, 45% length 2; SNOWY: 1
  drop->progress = 0;
  drop->nextupdate = now + drop->speed;
  illuminate(drop->lane * 5); // illuminate top of column
}

// check in with rain or snow drops to advance them or launch a new one
void processDropTimeDelays(unsigned long now) {
  // occasionally launch another drop
  if (now >= nextlaunchtime) {
    launchDrop(now, &travelers[nexttravelerid]);
    nextlaunchtime = getNextLaunchTime(now);
    nexttravelerid = (nexttravelerid + 1) % TRAVELERBUFSIZE;
  }
  // check traveler structs to advance drops
  for (uint8_t i = 0; i < TRAVELERBUFSIZE; i++)
    if (travelers[i].lane != -1 && travelers[i].nextupdate <= now)
      advanceDrop(&travelers[i]);
}

// #####################
// SNAKEY MODE FUNCTIONS
// #####################

// On slightly randomized intervals for each row, launch a horizontal "snake" of random length and speed to travel in either direction
// Only 5 traveler structs are used, one for each lane, and only one traveler runs through a lane at a time

// within each lane, downtime between snakes
unsigned long getIdleTime(unsigned long now) {
  return now + 500 + (rand() % 2500);  // between 1/2 second and 3 seconds
}

// advances snake one step along lane. Resets and reschedules traveler struct when lane is clear
void advanceSnake(struct traveler * snake) {
  snake->progress++;
  if (snake->progress <= 5)
    illuminate((snake->direction ? snake->progress : 5 - snake->progress) * 5 + snake->lane); // advance front of snake in whichever direction
  int tailprog = snake->progress - snake->length; // position of tail of snake to be deluminated
  if (tailprog >= 0 && tailprog <= 5) // tail is on board
    deluminate((snake->direction ? tailprog : 5 - tailprog) * 5 + snake->lane, false); // release tail of snake
  if (tailprog >= 5) { // tail just dropped off board
    snake->speed = -1; // mark lane as idle
    snake->nextupdate = getIdleTime(snake->nextupdate); // schedule next snake launch in this lane
  } else {
    snake->nextupdate += snake->speed; // schedule next advancement
  }
}

// launches a rain or snow drop from the top of a random column
void launchSnake(unsigned long now, struct traveler * snake) {
  snake->direction = rand() % 2; // 0 means right-to-left, 1 means left-to-right
  snake->speed = 200 + (rand() % 130); // 200 to 330 ms per horizontal step 
  snake->length = 2 + (rand() % 4); // 2-5 segments in length
  snake->progress = 0;
  snake->nextupdate = now + snake->speed;
  illuminate(snake->lane + (snake->direction ? 0 : 25)); // illuminate beginning of snake (right or left edge of board)
}

void startSnakeyMode() {
  unsigned long now = millis();
  for (int i = 0; i < 5; i++) {
    travelers[i].lane = i;
    travelers[i].speed = -1; // indicator that lane is currently idle
    travelers[i].nextupdate = now + (rand() % 3000); // stagger the start times
  }
}

// checks in with traveler structs to advance snakes and launch new ones
void processSnakeTimeDelays(unsigned long now) {
  for (int i = 0; i < 5; i ++) {
    if (travelers[i].nextupdate <= now) {
      if (travelers[i].speed == -1)
        launchSnake(now, &travelers[i]);
      else 
        advanceSnake(&travelers[i]);
    }
  }
}


// #####################
// METEOR MODE FUNCTIONS
// #####################

// On sporadic intervals, launch a meteor that starts on the top or right edge of matrix and travels diagonally
// down and to the left. Direction flag in traveler indicates travel angle 0 (shallow) or 1 (steep). 9 lanes are
// available, starting from 0 (top second from left) to 4 (top-right corner) to 8 (bottom-right corner).

void startMeteorMode() {
  for (int i = 0; i < TRAVELERBUFSIZE; i++)
    travelers[i].lane = -1; // mark all entries as invalid
  nextlaunchtime = millis() + 300 + (rand() % 1000);
}

// checks in with meteor structs to advance meteors and launch new ones
void processMeteorTimeDelays(unsigned long now) {
  // occasionally launch another meteor
  if (now >= nextlaunchtime) {
    launchMeteor(now, &travelers[nexttravelerid]);
    nextlaunchtime = getNextLaunchTime(now);
    nexttravelerid = (nexttravelerid + 1) % TRAVELERBUFSIZE;
  }
  // check meteor structs to advance meteors
  for (uint8_t i = 0; i < TRAVELERBUFSIZE; i++)
    if (travelers[i].lane != -1 && travelers[i].nextupdate <= now)
      advanceMeteor(&travelers[i]);
}

// launches a meteor from the edge of the matrix
void launchMeteor(unsigned long now, struct traveler * meteor) {
  meteor->direction = rand() % 2; // 0 is shallow angle, 1 is steep
  meteor->lane = rand() % 9; // starting position: 0-4 are top row (not including top left), 4-8 are right edge
  meteor->speed = (50 + (rand() % 100)); // 50-150 ms per step
  meteor->length = (meteor->direction ? 2 : 2 + (rand() % 2)); // steep meteors are length 2, shallow meteors are length 2 or 3
  meteor->progress = 0;
  meteor->row = meteor->lane < 5 ? 0 : meteor->lane - 4;
  meteor->col = meteor->lane > 3 ? 5 : meteor->lane + 1;
  meteor->nextupdate = now + meteor->speed;
  illuminate(meteor->lane < 5 ? (meteor->lane + 1) * 5 : meteor->lane + 21); // illuminate starting position
  // Serial.println("LAUNCHING: row, col, pos: ");
  // Serial.println(meteor->row);
  // Serial.println(meteor->col);
  // Serial.println(meteor->lane < 5 ? (meteor->lane + 1) * 5 : meteor->lane + 21);
  // Serial.println(meteor->direction ? "STEEP" : "SHALLOW");
  nextlaunchtime = getNextLaunchTime(now);
}

// advances meteor one step along lane. Resets traveler struct when meteor advances off the board
void advanceMeteor(struct traveler * meteor) {
  meteor->progress++;
  meteor->row += ((!(meteor->col % 2)) + meteor->direction);  // increment row. shallows drop by 0 or 1, steeps drop by 1 or 2, depending on parity of col
  meteor->col--; // decrement col
  if (meteor->row < 5 && meteor->col >= 0) {// if leading edge still on board, illuminate
    illuminate(meteor->col * 5 + meteor->row);
    // Serial.println("ADVANCING to pos:");
    // Serial.println(meteor->col * 5 + meteor->row);
  }
  int8_t tailcol = meteor->col + meteor->length; // get tail position
  int8_t tailrow = meteor->row - (meteor->direction ? 3 : (meteor->length == 3 && (meteor->col % 2) ? 2 : 1));
  // shallow, new front is even col, length 2		:   new front row - 1
  // shallow, new front is odd col, length 2		:   new front row - 1
  // shallow, new front is even col, length 3       :   new front row - 1
  // shallow, new front is odd col, length 3		:   new front row - 2
  // steep, new front is even col, length 2			:   new front row - 3
  // steep, new front is odd col, length 2			:   new front row - 3
  	//   Serial.println("CALCULATED POTENTIAL DELUMINATE row, col, pos:");
	  // Serial.println(tailrow);
	  // Serial.println(tailcol);
  	//   Serial.println(tailcol * 5 + tailrow);
  if (tailrow >= 0 && tailrow <= 4 && tailcol >= 0 && tailcol <= 5)
	  deluminate(tailcol * 5 + tailrow, false); // deluminate tail
	  // Serial.println("ACTUALLY DELUMINATING");
  if ((tailrow >= 4 && (meteor->direction || !(tailcol % 2))) || tailcol <= 0) // if tail just dropped off board, reset meteor
  	// dropped off when tail is at/past far left col, or on/below bottom row as long as either steep or even-column
    meteor->lane = -1;
  else // else, schedule next advancement
    meteor->nextupdate += meteor->speed;
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
    } else if (pianomode == COLUMNS) {
      resetColMode();
      pianomode = STARRY;
    }
  } else if (globalmode == AUTO) {
    if (automode == ALLON) {
      resetAllOnMode();
      automode = RAINY;
      startDropMode();
    } else if (automode == RAINY) {
      resetTravelerMode();
      automode = SNOWY;
      startDropMode();
    } else if (automode == SNOWY) {
      resetTravelerMode();
      automode = SNAKEY;
      startSnakeyMode();
    } else if (automode == SNAKEY) {
      resetTravelerMode();
      automode = METEOR;
      startMeteorMode();
    } else if (automode == METEOR) {
      resetTravelerMode();
      automode = ALLON;
      startAllOn();
    }
  }
}

// cycle the other direction
void changemodedown() {
  if (globalmode == PIANO) {
    changemodeup(); // only two piano modes currently
  } else if (globalmode == AUTO) {
    if (automode == ALLON) {
      resetAllOnMode();
      automode = METEOR;
      startMeteorMode();
    } else if (automode == RAINY) {
      resetTravelerMode();
      automode = ALLON;
      startAllOn();
    } else if (automode == SNOWY) {
      resetTravelerMode();
      automode = RAINY;
      startDropMode();
    } else if (automode == SNAKEY) {
      resetTravelerMode();
      automode = SNOWY;
      startDropMode();
    } else if (automode == METEOR) {
      resetTravelerMode();
      automode = SNAKEY;
      startSnakeyMode();
    }
  }
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
    } else if (automode == RAINY || automode == SNOWY) {
      startDropMode();
    } else if (automode == SNAKEY) {
      startSnakeyMode();
    } else if (automode == METEOR) {
      startMeteorMode();
    }
  } else {
    Serial.println("Switch going up but already in auto mode. Should not get here");
  }
}

// toggling switch from up to down (into piano mode)
void switchDown() {
  if (globalmode == AUTO) {
    if (automode == RAINY || automode == SNOWY || automode == SNAKEY || automode == METEOR) {
      resetTravelerMode();
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
    if (automode == RAINY || automode == SNOWY)
      processDropTimeDelays(now);
    else if (automode == SNAKEY)
      processSnakeTimeDelays(now);
    else if (automode == METEOR)
      processMeteorTimeDelays(now);
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

 // !! This does not register when the switch starts in the down position. Currently not an issue because piano modes don't require startup and I think globalmode defaults to piano (just by being first (0) in the enum list), but it will cause issues down the line.
// 7/3/20 ^^ I don't know if this is still the case or if it registers because of being called at startup
void pollSwitch() {
  unsigned long now = millis();
  int reading = digitalRead(SWITCHPIN);
  if (reading != lastSwitchState) {
    lastSwitchDebounceTime = now;
  }
  if ((now - lastSwitchDebounceTime) > DEBOUNCEDELAY) {
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

// ARDUINO SETUP
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
  // Serial.println(freeRam());
}

// ARDUINO LOOP
void loop() {
  processButtons();
  processMidi();
  processTimeDelays();
  delay(2); // !! rather than delay, maybe just use millis to poll the midi source every 2 or 3 ms.
  // if (millis() % 2500 == 0 || millis() % 2500 == 1)
  //   Serial.println(freeRam());
}


// could adjust by range: high (melody) notes get solo treatment, while bass notes are bundled to avoid clutter

// todo:
// tune columns intensities (especially soft ones) to match actual intensity. Softest threshold is too soft
// tune colunmns horizontal interval widths to match common playing intervals, kind of like thirds in starry mode
// make columns re-flash repeated sustained columns like starry mode. Maybe also reflash decaying notes that are still on (I will get that for free I think)
// lucinda says sustained columns should decay faster so you can actually see them, and also maybe fire up as well

