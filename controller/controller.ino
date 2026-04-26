#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

/* ===================== UART FROM ESP32 ===================== */
static const int ESP_RX_PIN = 10;   // UNO RX from ESP32 TX
static const int ESP_TX_PIN = 11;   // unused
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);

/* ===================== LCD ===================== */
LiquidCrystal_I2C lcd(0x27, 16, 2);

/* ===================== SERVO ===================== */
static const int PIN_LID_SERVO = 3;
static const int LID_CLOSED = 180;
static const int LID_OPEN   = 90;

static const unsigned long OPEN_SETTLE_MS  = 300;
static const unsigned long DROP_TIME_MS    = 1500;
static const unsigned long CLOSE_SETTLE_MS = 400;

/* ===== HARD BREAK AFTER CLOSING ===== */
static const unsigned long POST_CLOSE_BREAK_MS = 5000;

/* ===================== BUZZER + BUTTON + MOSFET ===================== */
static const int PIN_BUZZER = 6;      // passive/active buzzer
static const int PIN_REDEEM = 7;      // button to GND (INPUT_PULLUP)
static const int PIN_MOSFET = 8;      // MOSFET/relay control (HIGH = ON)

static const unsigned long DEBOUNCE_MS = 50;

/* ===================== CREDITS / CHARGING ===================== */
static const uint16_t MINUTES_PER_CREDIT = 10;
static const uint32_t MS_PER_MINUTE = 60000UL;

static volatile uint16_t creditsTotal = 0;      // session credits only
static bool charging = false;
static uint32_t chargeEndMs = 0;                // millis() time when charging ends
static uint32_t lastScreenMs = 0;               // for screen refresh throttling

/* ===================== LID STATE ===================== */
enum LidState {
  LID_IDLE,
  LID_OPENING,
  LID_WAIT_DROP,
  LID_CLOSING,
  LID_BREAK
};

static LidState lidState = LID_IDLE;
static unsigned long lidTs = 0;

Servo lidServo;

/* ===================== RX PARSER (DUAL MODE) ===================== */
// Supports BOTH:
//  1) newline mode: "plastic_bottle\n"
//  2) framed mode : "#plastic_bottle!"
static bool inFrame = false;
static char buf[40];
static uint8_t len = 0;

static inline bool lidBusy() { return lidState != LID_IDLE; }

/* ===================== HELPERS ===================== */
static void flushSerial() {
  while (espSerial.available()) espSerial.read();
  len = 0;
  inFrame = false;
}

static void buzzerOff() { noTone(PIN_BUZZER); }

/* plastic_bottle melody: beep beep beep (nice "good" tone) */
static void beepPlasticMelody() {
  // Short rising triple (loud perception comes from higher freq)
  const uint16_t f1 = 1568; // G6
  const uint16_t f2 = 1760; // A6
  const uint16_t f3 = 2093; // C7

  tone(PIN_BUZZER, f1, 120); delay(140);
  tone(PIN_BUZZER, f2, 120); delay(140);
  tone(PIN_BUZZER, f3, 170); delay(190);
  buzzerOff();
}

/* redeem: one long beep 1000ms */
static void beepRedeemLong() {
  tone(PIN_BUZZER, 2000, 1000);
  delay(1000);
  buzzerOff();
}

/* ===================== LCD SCREENS ===================== */
static void lcdShowWelcome() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Welcome");
  lcd.setCursor(0,1); lcd.print("Insert bottle");
}

static void lcdShowCreditsPressRedeem() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Credits: ");
  lcd.print(creditsTotal);
  lcd.setCursor(0,1); lcd.print("Press Redeem");
}

static void formatHHMMSS(uint32_t secs, char out[9]) {
  uint32_t h = secs / 3600UL;
  uint32_t m = (secs % 3600UL) / 60UL;
  uint32_t s = secs % 60UL;
  // HH:MM:SS (always 2 digits)
  out[0] = '0' + (h / 10) % 10;
  out[1] = '0' + (h % 10);
  out[2] = ':';
  out[3] = '0' + (m / 10);
  out[4] = '0' + (m % 10);
  out[5] = ':';
  out[6] = '0' + (s / 10);
  out[7] = '0' + (s % 10);
  out[8] = '\0';
}

static void lcdShowCharging(uint16_t totalMins, uint32_t msLeft) {
  uint32_t secsLeft = (msLeft + 999UL) / 1000UL; // ceil
  char t[9];
  formatHHMMSS(secsLeft, t);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Charging: ");
  lcd.print(totalMins);
  lcd.print("min");

  lcd.setCursor(0,1);
  lcd.print(t);
}

/* ===================== MOSFET CONTROL ===================== */
static void setMosfet(bool on) {
  digitalWrite(PIN_MOSFET, on ? HIGH : LOW);
}

/* ===================== LID CONTROL (NON-BLOCKING) ===================== */
static void startLidSequence() {
  lidState = LID_OPENING;
  lidTs = millis();
  lidServo.write(LID_OPEN);
}

static void updateLidSequence() {
  unsigned long now = millis();

  switch (lidState) {
    case LID_IDLE: return;

    case LID_OPENING:
      if (now - lidTs >= OPEN_SETTLE_MS) { lidState = LID_WAIT_DROP; lidTs = now; }
      break;

    case LID_WAIT_DROP:
      if (now - lidTs >= DROP_TIME_MS) {
        lidServo.write(LID_CLOSED);
        lidState = LID_CLOSING;
        lidTs = now;
      }
      break;

    case LID_CLOSING:
      if (now - lidTs >= CLOSE_SETTLE_MS) {
        lidState = LID_BREAK;
        lidTs = now;
        flushSerial(); // kill historical data immediately
      }
      break;

    case LID_BREAK:
      if (now - lidTs >= POST_CLOSE_BREAK_MS) {
        lidState = LID_IDLE;
        flushSerial(); // flush again before re-arming
      }
      break;
  }
}

/* ===================== SYSTEM STATE UPDATES ===================== */
static void refreshScreens() {
  // throttle LCD updates to avoid flicker
  unsigned long now = millis();
  if (now - lastScreenMs < 250) return;
  lastScreenMs = now;

  if (charging) {
    // remaining time (handle millis overflow safely using signed subtraction)
    int32_t diff = (int32_t)(chargeEndMs - now);
    if (diff <= 0) {
      // charging finished
      charging = false;
      setMosfet(false);
      creditsTotal = 0;
      lcdShowWelcome();
      return;
    }

    uint32_t msLeft = (uint32_t)diff;
    uint16_t totalMins = (uint16_t)( (uint32_t)creditsTotal * MINUTES_PER_CREDIT );
    // If creditsTotal was changed elsewhere (shouldn't while charging), still display based on stored creditsTotal.
    lcdShowCharging(totalMins, msLeft);
  } else {
    if (creditsTotal == 0) lcdShowWelcome();
    else lcdShowCreditsPressRedeem();
  }
}

static void startChargingFromCredits() {
  if (creditsTotal == 0) return;

  charging = true;

  uint32_t totalMinutes = (uint32_t)creditsTotal * MINUTES_PER_CREDIT;
  uint32_t totalMs = totalMinutes * MS_PER_MINUTE;

  chargeEndMs = millis() + totalMs;

  // MOSFET ON only while time left > 0
  setMosfet(true);

  // immediate screen
  lcdShowCharging((uint16_t)totalMinutes, totalMs);
}

/* ===================== HANDLE MESSAGE ===================== */
static void handleMessage(const char* msg) {
  // ONLY print what is received
  Serial.println(msg);

  // ignore everything while charging
  if (charging) return;

  // ignore while lid moving + break
  if (lidBusy()) return;

  // accept only plastic_bottle
  if (strcmp(msg, "plastic_bottle") != 0) return;

  creditsTotal++;

  // show credits screen (press redeem)
  lcdShowCreditsPressRedeem();

  // buzzer melody for valid bottle
  beepPlasticMelody();

  // open lid to drop bottle
  startLidSequence();
}

/* ===================== FEED ONE CHAR INTO PARSER ===================== */
static void feedChar(char c) {
  if (c == '\r') return;

  if (c == '#') { inFrame = true; len = 0; return; }

  if (c == '!' && inFrame) {
    if (len > 0) { buf[len] = '\0'; handleMessage(buf); }
    inFrame = false; len = 0;
    return;
  }

  if (c == '\n' && !inFrame) {
    if (len > 0) { buf[len] = '\0'; handleMessage(buf); }
    len = 0;
    return;
  }

  if (len < sizeof(buf) - 1) {
    if (c >= 'A' && c <= 'Z') c = (char)(c + 32);
    buf[len++] = c;
  } else {
    len = 0;
    inFrame = false;
  }
}

/* ===================== REDEEM BUTTON ===================== */
static bool redeemPrev = HIGH;
static unsigned long redeemTs = 0;
static bool redeemArmed = true;

static void updateRedeemButton() {
  bool nowState = digitalRead(PIN_REDEEM); // INPUT_PULLUP: pressed = LOW

  if (nowState != redeemPrev) {
    redeemTs = millis();
    redeemPrev = nowState;
  }

  if ((millis() - redeemTs) < DEBOUNCE_MS) return;

  // press edge
  if (redeemArmed && nowState == LOW) {
    redeemArmed = false;

    // If charging, do nothing (rule #5)
    if (charging) return;

    // If no credits, do nothing (still on welcome)
    if (creditsTotal == 0) return;

    // Start charging (rule #4)
    beepRedeemLong();
    startChargingFromCredits();
  }

  if (nowState == HIGH) redeemArmed = true;
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);
  espSerial.begin(9600);

  pinMode(PIN_BUZZER, OUTPUT);
  buzzerOff();

  pinMode(PIN_REDEEM, INPUT_PULLUP);

  pinMode(PIN_MOSFET, OUTPUT);
  setMosfet(false); // default OFF

  lidServo.attach(PIN_LID_SERVO);
  lidServo.write(LID_CLOSED);

  lcd.init();
  lcd.backlight();

  creditsTotal = 0;
  charging = false;
  lcdShowWelcome();

  flushSerial();
}

/* ===================== LOOP ===================== */
void loop() {
  updateLidSequence();

  // Update redeem button always
  updateRedeemButton();

  // While charging:
  //  - ignore ESP32 messages
  //  - keep MOSFET on only if time left > 0
  //  - update countdown screen
  if (charging) {
    flushSerial();      // prevent backlog while charging
    refreshScreens();   // updates countdown + handles end
    return;
  }

  // While lid/break: ignore + flush so no historical backlog remains
  if (lidBusy()) {
    flushSerial();
    refreshScreens();
    return;
  }

  // Normal: read bytes and parse
  while (espSerial.available()) {
    char c = (char)espSerial.read();
    feedChar(c);
  }

  // Keep welcome/credits screen updated (no flicker)
  refreshScreens();
}
