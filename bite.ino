/*
 * Arduino Mega – Bite Force Rig Firmware Skeleton (MOCK MODE)
 * -----------------------------------------------------------
 * Purpose: Run the full control/data flow without hardware.
 * When parts arrive, flip MOCK_MODE to 0 and drop in real drivers.
 *
 * Board: Arduino Mega 2560
 * Serial: 115200 baud (use Serial Monitor/Plotter)
 * Commands: HELP | STATUS | TARE | RUN STEP <N> <HOLD_MS> | STOP
 *
 * Pin plan (adjust later):
 *  - HX711: DT=22, SCK=23  (unused in mock)
 *  - Valves: EXTEND=24, RETRACT=25 (mock = onboard LED blink)
 *  - FSRs: A0 (left), A1 (right)  (unused in mock)
 *  - E-Stop: 26 (INPUT_PULLUP)
 */

/************* CONFIG *************/
#define MOCK_MODE 1           // 1 = simulate sensors/valves, 0 = real hardware
#define BAUD_RATE 115200

// Pins
const uint8_t PIN_LC_DT   = 22;  // HX711
const uint8_t PIN_LC_SCK  = 23;  // HX711
const uint8_t PIN_VALVE_EXTEND = 24;
const uint8_t PIN_VALVE_RETRACT = 25;
const uint8_t PIN_FSR_L   = A0;
const uint8_t PIN_FSR_R   = A1;
const uint8_t PIN_ESTOP   = 26;  // Active LOW with INPUT_PULLUP

// Safety & control
const float   F_MAX_N        = 650.0;  // Hard cutout
const uint32_t AUTO_VENT_MS  = 3000;   // Max continuous pressurize without profile

// Sampling/logging
const uint16_t SAMPLE_HZ = 100;        // 100 samples/sec target

/************* TYPES *************/
enum RigState : uint8_t {
  IDLE=0, TARE, READY, PRESSURIZE, HOLD, VENT, RUNNING, FAULT
};

struct Sample {
  uint32_t t_ms;
  float    lc_N;      // load cell estimate [N]
  int      fsrL_raw;  // 0..1023
  int      fsrR_raw;  // 0..1023
  uint8_t  valve_ext; // 0/1
  uint8_t  valve_ret; // 0/1
};

/************* GLOBALS *************/
RigState g_state = IDLE;
uint32_t g_lastSampleMs = 0;
uint32_t g_lastValveChangeMs = 0;

// RUN profile params
float    g_targetN = 0;
uint32_t g_holdMs  = 0;
uint32_t g_runStartMs = 0;

/************* MOCK DRIVERS *************/
#if MOCK_MODE
// Simple pseudo-random but smooth-ish signal generator
float mockLoadCell(float targetN) {
  // Approach target with a first-order response and add ripple
  static float y = 0;
  float err = targetN - y;
  y += 0.08f * err;                     // response speed
  float ripple = 5.0f * sinf(0.02f * millis());
  float noise  = ((random(0, 100) - 50) / 50.0f); // ±1
  float val = max(0.0f, y + ripple + noise);
  // hard limit to F_MAX_N
  if (val > F_MAX_N) val = F_MAX_N;
  return val;
}

int mockFSR(int channel, float lcN) {
  // Split LC between left/right with small bias that drifts over time
  float bias = 0.5f + 0.05f * sinf(0.0015f * millis()); // 0.45..0.55
  float leftN  = lcN * bias;
  float rightN = lcN * (1.0f - bias);
  float chosen = (channel==0) ? leftN : rightN;
  // Map ~0..600 N → ~100..900 ADC to mimic divider
  int adc = (int)constrain(100 + chosen * (800.0f/600.0f), 0, 1023);
  return adc;
}

void mockValveWrite(uint8_t pin, uint8_t val){
  // In mock, just track time and optionally blink onboard LED when extending
  if(pin==PIN_VALVE_EXTEND && val==HIGH){
    // blink LED 13 quickly to show EXTEND
    digitalWrite(LED_BUILTIN, (millis()/100)%2);
  }
}
#else
// TODO: Replace with real HX711 & digitalWrite for valves
#include <HX711.h>
HX711 scale;
float realLoadCell(){
  long raw = scale.read_average(4);
  // Apply calibration: N = slope*raw + offset (to be set after calibration)
  const float slope = 1.0f; // placeholder
  const float offs  = 0.0f; // placeholder
  float n = slope * raw + offs;
  if (n < 0) n = 0;
  return n;
}
#endif

/************* VALVE CONTROL (common) *************/
uint8_t g_valveExt=0, g_valveRet=0;

void setValves(uint8_t ext, uint8_t ret){
  g_valveExt = ext ? 1 : 0;
  g_valveRet = ret ? 1 : 0;
  g_lastValveChangeMs = millis();
#if MOCK_MODE
  mockValveWrite(PIN_VALVE_EXTEND, g_valveExt);
  mockValveWrite(PIN_VALVE_RETRACT, g_valveRet);
#else
  digitalWrite(PIN_VALVE_EXTEND, g_valveExt);
  digitalWrite(PIN_VALVE_RETRACT, g_valveRet);
#endif
}

void allVent(){ setValves(0,1); }
void allOff(){ setValves(0,0); }
void pressurize(){ setValves(1,0); }

/************* UTIL *************/
bool estopActive(){ return digitalRead(PIN_ESTOP)==LOW; }

void printHeader(){
  Serial.println("t_ms,lc_N,fsrL_raw,fsrR_raw,valve_ext,valve_ret");
}

void printHelp(){
  Serial.println(F("Commands:"));
  Serial.println(F("  HELP"));
  Serial.println(F("  STATUS"));
  Serial.println(F("  TARE"));
  Serial.println(F("  RUN STEP <N> <HOLD_MS>"));
  Serial.println(F("  STOP"));
}

/************* COMMAND PARSER *************/
String inbuf;

void handleLine(String s){
  s.trim(); s.toUpperCase();
  if(s=="HELP"){ printHelp(); return; }
  if(s=="STATUS"){ Serial.print(F("STATE=")); Serial.println((int)g_state); return; }
  if(s=="TARE"){ g_state=TARE; Serial.println(F("Taring... (mock)")); delay(50); g_state=READY; Serial.println(F("Ready.")); return; }
  if(s.startsWith("RUN STEP")){
    // Format: RUN STEP <N> <HOLD_MS>
    s.replace("RUN STEP"," "); s.trim();
    float N = s.toFloat();
    int idx = s.indexOf(' ');
    uint32_t hold = 1000;
    if(idx>0){ hold = s.substring(idx+1).toInt(); }
    g_targetN = N; g_holdMs = hold; g_runStartMs = millis();
    g_state = RUNNING; Serial.print(F("RUN STEP targetN=")); Serial.print(g_targetN); Serial.print(F(" holdMs=")); Serial.println(g_holdMs);
    printHeader();
    return;
  }
  if(s=="STOP"){ g_state=VENT; Serial.println(F("Stopping → VENT")); return; }
  Serial.println(F("? Unknown command. Type HELP"));
}

void pollSerial(){
  while(Serial.available()){
    char c = Serial.read();
    if(c=='\n' || c=='\r'){
      if(inbuf.length()>0){ handleLine(inbuf); inbuf=""; }
    } else {
      inbuf += c;
    }
  }
}

/************* SETUP *************/
void setup(){
  Serial.begin(BAUD_RATE);
  pinMode(PIN_VALVE_EXTEND, OUTPUT);
  pinMode(PIN_VALVE_RETRACT, OUTPUT);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  allOff();
#if !MOCK_MODE
  // HX711 init example (DT,SCK)
  // scale.begin(PIN_LC_DT, PIN_LC_SCK);
  // scale.set_gain(128);
#endif
  g_state = READY;
  Serial.println(F("Bite Rig Firmware (MOCK MODE) ready. Type HELP."));
}

/************* MAIN LOOP *************/
void loop(){
  pollSerial();

  // Safety: E-stop hard cut
  if(estopActive()){
    if(g_state!=FAULT){ Serial.println(F("E-STOP! Venting.")); }
    g_state = FAULT; allVent();
  }

  // State machine
  switch(g_state){
    case READY: {
      // Idle, waiting for RUN command
      allOff();
      break;
    }
    case RUNNING: {
      // Simple STEP profile: pressurize, hold, vent
      uint32_t t = millis() - g_runStartMs;
      if(t < g_holdMs){
        pressurize();
      } else {
        g_state = VENT;
        allVent();
      }
      break;
    }
    case VENT: {
      // Stay venting for 500 ms then go READY
      static uint32_t ventStart=0; if(ventStart==0) ventStart=millis();
      if(millis()-ventStart>500){ ventStart=0; g_state=READY; allOff(); }
      break;
    }
    case FAULT: {
      // Vent until user clears (reset board or add a CLEAR cmd later)
      allVent();
      break;
    }
    default: break;
  }

  // Auto safety: don't allow pressurize forever outside RUNNING
  if(g_state!=RUNNING && g_valveExt && (millis()-g_lastValveChangeMs)>AUTO_VENT_MS){ allVent(); }

  // Sampling/logging at SAMPLE_HZ
  uint32_t now = millis();
  if(now - g_lastSampleMs >= (1000UL / SAMPLE_HZ)){
    g_lastSampleMs = now;
    float lcN;
#if MOCK_MODE
    // Aim for target when running, otherwise near 0 with a little noise
    float aim = (g_state==RUNNING) ? g_targetN : 0.0f;
    lcN = mockLoadCell(aim);
#else
    lcN = realLoadCell();
#endif
    int fsrL = mockFSR(0, lcN); // in MOCK, derive from lcN
    int fsrR = mockFSR(1, lcN);

    // Cap at F_MAX_N for safety
    if(lcN > F_MAX_N){
      Serial.println(F("Force limit! Venting."));
      g_state = FAULT; allVent();
    }

    // Log CSV during RUN and READY for testing
    if(g_state==RUNNING || g_state==READY){
      Serial.print(now); Serial.print(',');
      Serial.print(lcN, 1); Serial.print(',');
      Serial.print(fsrL); Serial.print(',');
      Serial.print(fsrR); Serial.print(',');
      Serial.print(g_valveExt); Serial.print(',');
      Serial.println(g_valveRet);
      
    }
    
  }
}
