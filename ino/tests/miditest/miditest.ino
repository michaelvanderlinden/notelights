#include <usbh_midi.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
USBH_MIDI Midi(&Usb);

// unfortunately I can't use pin 13 to signal LED because USB uses pin 13
void setup() {
  Serial.begin(57600);
  if (Usb.Init() == -1) {
    Serial.write("USB failed to init");
    while (1); // halt
  }
  Serial.write("USB initialized successfully!");
  delay(200);
}

void loop() {
  Usb.Task();
  MIDI_poll();
  delay(1);
}

// Poll USB MIDI Controller and send to serial
void MIDI_poll() {
  uint8_t outBuf[3];
  uint8_t bsize;
  int dataexists = 0;
  do {
    if ((bsize = Midi.RecvData(outBuf)) > 0) {
      dataexists = 1;
      for (int i = 0; i < bsize; i++) {
        Serial.print(outBuf[i], HEX);
        Serial.print(" ");
      }
    }
  } while (bsize > 0);
  if (dataexists) {
    Serial.print("\n");
  }
}
