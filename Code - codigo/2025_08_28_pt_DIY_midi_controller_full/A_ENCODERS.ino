/////////////////////////////////////////////
// ENCODERS

#ifdef USING_ENCODER  // only compiled if encoders are defined in the setup

// -------------------------------
// Variables you don't need to change
// -------------------------------
int lastEncoderValue[N_ENCODER_MIDI_CHANNELS][N_ENCODERS] = { 0 };
int lastEncoderMidiValue[N_ENCODER_MIDI_CHANNELS][N_ENCODERS] = { 0 };
int encoderValue[N_ENCODER_MIDI_CHANNELS][N_ENCODERS] = { 0 };
int encoderMidiValue[N_ENCODER_MIDI_CHANNELS][N_ENCODERS] = { 0 };
int encoderMackieValue[N_ENCODER_MIDI_CHANNELS][N_ENCODERS] = { 0 };

byte lastEncoderChannel = 0;

unsigned long encPTime[N_ENCODERS] = { 0 };
unsigned long encTimer[N_ENCODERS] = { 0 };
boolean encMoving[N_ENCODERS] = { false };
boolean encMoved[N_ENCODERS] = { false };
byte encTIMEOUT = 50;
byte encoder_n;
float encTempVal = 0;

#ifdef USING_HIGH_RES_ENC
unsigned int highResenc[N_ENCODER_MIDI_CHANNELS][N_ENCODERS] = { 0 };
byte encMSB[N_ENCODER_MIDI_CHANNELS][N_ENCODORS] = { 0 };
byte encLSB[N_ENCODER_MIDI_CHANNELS][N_ENCODORS] = { 0 };
byte pencLSB[N_ENCODER_MIDI_CHANNELS][N_ENCODORS] = { 0 };
#endif

// -------------------------------
// Software filtering controls
// -------------------------------
const uint8_t ENC_STEPS_PER_DETENT = 4;   // 1, 2 or 4
const uint8_t ENC_DEADTIME_MS      = 3;   // 2–8ms typical
const uint8_t ENC_IDLE_HYSTERESIS  = 1;   // ignore +/-1 twitch

long encPrevRaw[N_ENCODERS]              = { 0 };
unsigned long encLastEventMs[N_ENCODERS] = { 0 };

// -------------------------------
// Main function
// -------------------------------
void encoders() {

  // 1) Read and pre-process raw values
  for (int i = 0; i < N_ENCODERS; i++) {

    long raw = encoder[i].read();  // cumulative quadrature count

    // small hysteresis to avoid flicker at edges
    if (abs(raw - encPrevRaw[i]) <= ENC_IDLE_HYSTERESIS) {
      raw = encPrevRaw[i];
    }
    encPrevRaw[i] = raw;

    encoderValue[ENCODER_MIDI_CH][i] = (int)raw;

#if !defined(TRAKTOR) && !defined(USING_MACKIE)
    clipEncoderValue(i, encoderMinVal, encoderMaxVal * encSensitivity);
#endif

#ifdef USING_HIGH_RES_ENC
    highResenc[ENCODER_MIDI_CH][i] = map(
      encoderValue[ENCODER_MIDI_CH][i],
      0,  encoderMaxVal * encSensitivity,
      0,  encoderMaxVal
    );
    encMSB[ENCODER_MIDI_CH][i] = highResenc[ENCODER_MIDI_CH][i] / 128;
    encLSB[ENCODER_MIDI_CH][i] = highResenc[ENCODER_MIDI_CH][i] % 128;
#endif

    encoderMidiValue[ENCODER_MIDI_CH][i] = map(
      encoderValue[ENCODER_MIDI_CH][i],
      encoderMinVal,               encoderMaxVal * encSensitivity,
      0,                            encoderMaxVal
    );
  }

  // 2) Emit MIDI if something actually changed and passed filters
  for (int i = 0; i < N_ENCODERS; i++) {

    if (!encMoving[i]) {

#ifdef USING_HIGH_RES_ENC
      bool changed = (encLSB[ENCODER_MIDI_CH][i] != pencLSB[ENCODER_MIDI_CH][i]);
#else
      bool changed = (encoderMidiValue[ENCODER_MIDI_CH][i] != lastEncoderMidiValue[ENCODER_MIDI_CH][i]);
#endif

      if (changed) {

#ifdef TRAKTOR
        { // Relative 01/7F
          long delta = encoder[i].read();          // relative since last write(0)
          unsigned long now = millis();
          bool time_ok = (now - encLastEventMs[i] >= ENC_DEADTIME_MS);

          if (time_ok && (delta >= ENC_STEPS_PER_DETENT)) {
            encoderMidiValue[ENCODER_MIDI_CH][i] = 1;   // CW -> 0x01
            encoder[i].write(0);                        // consume
            encLastEventMs[i] = now;
          } else if (time_ok && (delta <= -ENC_STEPS_PER_DETENT)) {
            encoderMidiValue[ENCODER_MIDI_CH][i] = 127; // CCW -> 0x7F
            encoder[i].write(0);
            encLastEventMs[i] = now;
          } else {
            changed = false;                            // too small or still in deadtime
          }
        }
#endif // TRAKTOR

#ifdef USING_MACKIE
        if (changed) {
          long delta = encoder[i].read();
          unsigned long now = millis();
          bool time_ok = (now - encLastEventMs[i] >= ENC_DEADTIME_MS);

          if (time_ok && (delta >= ENC_STEPS_PER_DETENT)) {
            encoderMackieValue[ENCODER_MIDI_CH][i] = 0 + encoderSens;  // CW
            encoder[i].write(0);
            encLastEventMs[i] = now;
          } else if (time_ok && (delta <= -ENC_STEPS_PER_DETENT)) {
            encoderMackieValue[ENCODER_MIDI_CH][i] = 64 + encoderSens; // CCW
            encoder[i].write(0);
            encLastEventMs[i] = now;
          } else {
            changed = false;
          }
        }
#endif // USING_MACKIE

        if (changed) {

#ifdef ATMEGA328

  #ifdef USING_HIGH_RES_ENC
          MIDI.sendControlChange(ENCODER_CC_N[i], encMSB[ENCODER_MIDI_CH][i], ENCODER_MIDI_CH);
          MIDI.sendControlChange(ENCODER_CC_N[i] + 32, encLSB[ENCODER_MIDI_CH][i], ENCODER_MIDI_CH);
  #else
          MIDI.sendControlChange(ENCODER_CC_N[i], encoderMidiValue[ENCODER_MIDI_CH][i], ENCODER_MIDI_CH);
  #endif

#elif defined(ATMEGA32U4)

  #ifdef MIDI_DIN
    #ifdef USING_MACKIE
          controlChange(ENCODER_MIDI_CH, ENCODER_CC_N[i], encoderMackieValue[ENCODER_MIDI_CH][i]);  MidiUSB.flush();
          midi2.sendControlChange(ENCODER_CC_N[i], encoderMackieValue[ENCODER_MIDI_CH][i], ENCODER_MIDI_CH + 1);
    #elif defined(USING_HIGH_RES_ENC)
          controlChange(ENCODER_MIDI_CH, ENCODER_CC_N[i], encMSB[ENCODER_MIDI_CH][i]);  MidiUSB.flush();
          controlChange(ENCODER_MIDI_CH, ENCODER_CC_N[i] + 32, encLSB[ENCODER_MIDI_CH][i]);  MidiUSB.flush();
          midi2.sendControlChange(ENCODER_CC_N[i], encMSB[ENCODER_MIDI_CH][i], ENCODER_MIDI_CH + 1);
          midi2.sendControlChange(ENCODER_CC_N[i] + 32, encLSB[ENCODER_MIDI_CH][i], ENCODER_MIDI_CH + 1);
    #else
          controlChange(ENCODER_MIDI_CH, ENCODER_CC_N[i], encoderMidiValue[ENCODER_MIDI_CH][i]);  MidiUSB.flush();
          midi2.sendControlChange(ENCODER_CC_N[i], encoderMidiValue[ENCODER_MIDI_CH][i], ENCODER_MIDI_CH + 1);
    #endif
  #else
    #ifdef USING_MACKIE
          controlChange(ENCODER_MIDI_CH, ENCODER_CC_N[i], encoderMackieValue[ENCODER_MIDI_CH][i]);  MidiUSB.flush();
    #elif defined(USING_HIGH_RES_ENC)
          controlChange(ENCODER_MIDI_CH, ENCODER_CC_N[i], encMSB[ENCODER_MIDI_CH][i]);  MidiUSB.flush();
          controlChange(ENCODER_MIDI_CH, ENCODER_CC_N[i] + 32, encLSB[ENCODER_MIDI_CH][i]);  MidiUSB.flush();
    #else
          controlChange(ENCODER_MIDI_CH, ENCODER_CC_N[i], encoderMidiValue[ENCODER_MIDI_CH][i]);  MidiUSB.flush();
    #endif
  #endif

#elif defined(TEENSY)

  #ifdef USING_MACKIE
          usbMIDI.sendControlChange(ENCODER_CC_N[i], encoderMackieValue[ENCODER_MIDI_CH][i], ENCODER_MIDI_CH);
  #elif defined(USING_HIGH_RES_ENC)
          usbMIDI.sendControlChange(ENCODER_CC_N[i], encMSB[ENCODER_MIDI_CH][i], ENCODER_MIDI_CH);
          usbMIDI.sendControlChange(ENCODER_CC_N[i] + 32, encLSB[ENCODER_MIDI_CH][i], ENCODER_MIDI_CH);
  #else
          usbMIDI.sendControlChange(ENCODER_CC_N[i], encoderMidiValue[ENCODER_MIDI_CH][i], ENCODER_MIDI_CH);
  #endif

#elif defined(BLEMIDI)

  #ifdef USING_MACKIE
          BLEMidiServer.controlChange(ENCODER_MIDI_CH, ENCODER_CC_N[i], encoderMackieValue[ENCODER_MIDI_CH][i]);
  #elif defined(USING_HIGH_RES_ENC)
          BLEMidiServer.controlChange(ENCODER_MIDI_CH, ENCODER_CC_N[i], encMSB[ENCODER_MIDI_CH][i]);
          BLEMidiServer.controlChange(ENCODER_MIDI_CH, ENCODER_CC_N[i] + 32, encLSB[ENCODER_MIDI_CH][i]);
  #else
          BLEMidiServer.controlChange(ENCODER_MIDI_CH, ENCODER_CC_N[i], encoderMidiValue[ENCODER_MIDI_CH][i]);
  #endif

#endif // board paths

#ifdef DEBUG
          Serial.print("Encoder: "); Serial.print(i);
          Serial.print(" | ch: "); Serial.print(ENCODER_MIDI_CH);
          Serial.print(" | cc: "); Serial.print(ENCODER_CC_N[i]);
  #ifdef USING_MACKIE
          Serial.print(" | value: "); Serial.println(encoderMackieValue[ENCODER_MIDI_CH][i]);
  #elif defined(USING_HIGH_RES_ENC)
          Serial.print(" | HR: "); Serial.print(highResenc[ENCODER_MIDI_CH][i]);
          Serial.print(" | MSB: "); Serial.print(encMSB[ENCODER_MIDI_CH][i]);
          Serial.print(" | LSB: "); Serial.println(encLSB[ENCODER_MIDI_CH][i]);
  #else
          Serial.print(" | value: "); Serial.println(encoderMidiValue[ENCODER_MIDI_CH][i]);
  #endif
#endif

          lastEncoderMidiValue[ENCODER_MIDI_CH][i] = encoderMidiValue[ENCODER_MIDI_CH][i];
          lastEncoderValue[ENCODER_MIDI_CH][i]     = encoderValue[ENCODER_MIDI_CH][i];
#ifdef USING_HIGH_RES_ENC
          pencLSB[ENCODER_MIDI_CH][i]              = encLSB[ENCODER_MIDI_CH][i];
#endif
        } // changed -> send
      }   // if changed
    }     // if !encMoving
  }       // for i
}

// -------------------------------
// Helpers
// -------------------------------
void clipEncoderValue(int i, int minVal, int maxVal) {
  if (encoderValue[ENCODER_MIDI_CH][i] > maxVal - 1) {
    encoderValue[ENCODER_MIDI_CH][i] = maxVal;
    encoder[i].write(maxVal);
  }
  if (encoderValue[ENCODER_MIDI_CH][i] < minVal + 1) {
    encoderValue[ENCODER_MIDI_CH][i] = minVal;
    encoder[i].write(minVal);
  }
}

/* Prevent feedback while DAW is updating the encoder */
void isEncoderMoving() {
  for (int i = 0; i < N_ENCODERS; i++) {
    if (encMoved[i] == true) {
      encPTime[i] = millis();
    }
    encTimer[i] = millis() - encPTime[i];
    if (encTimer[i] < encTIMEOUT) {
      encMoving[i] = true;
      encMoved[i] = false;
    } else {
      if (encMoving[i] == true) {
        encMoving[i] = false;
      }
    }
  }
}

#endif  // USING_ENCODER
