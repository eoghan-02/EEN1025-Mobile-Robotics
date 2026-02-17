//Pin definitions
int Red1 = 4;
int Blue1 = 1;
int BUZZER_PIN = 2;


//Variables
unsigned long previousMillis = 0;
unsigned long interval = 200;
bool RedledState = LOW;
bool BlueledState = HIGH;
int ledCount = 0;

//=============SIREN SETUP====================

// ===== PWM carrier ==========
const uint32_t CARRIER_HZ = 20000;
const uint8_t  PWM_RES   = 10;

// ===== WAIL settings =====
const uint16_t DUTY_MIN = 75;
const uint16_t DUTY_MAX = 900;
const uint16_t STEP     = 8;
const uint32_t STEP_MS  = 2;

// ===== YELP settings =====c  
const uint32_t YELP_INTERVAL_MS = 4000;  // how often yelp occurs
const uint32_t YELP_DURATION_MS = 800;   // how long yelp lasts
const uint32_t YELP_PULSE_MS    = 30;    // yelp aggressiveness 

//====================================================================

void setup() {
  Serial.begin(115200);
  pinMode(Red1, OUTPUT);
  pinMode(Blue1,OUTPUT);
}

void loop() {
  flashing(true);
  sirenUpdate(true);
}

void flashing(bool enable) {

  if(!enable){
    digitalWrite(Red1, LOW);
    digitalWrite(Blue1, LOW);
    ledCount = 0;
    interval = 200;
    
    return;
  } 

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;      // save the last time
    RedledState = !RedledState;
    BlueledState = !BlueledState;                 // toggle state
    digitalWrite(Red1, RedledState);
    digitalWrite(Blue1, BlueledState);

    ledCount++;

    if (ledCount >= 5) {        
      interval = 50;            // quick bursts
    }
    if (ledCount >= 15) {       
      interval = 200;           // slower flash
      ledCount = 0;             // restart cycle
  }
  }
}

void sirenUpdate(bool enable) { 
  static bool init = false;

  // Wail state
  static uint16_t duty = DUTY_MIN;
  static int dir = +1;
  static uint32_t lastStep = 0;

  // Yelp state
  static bool yelpActive = false;
  static bool yelpOn = false;
  static uint32_t lastYelp = 0;
  static uint32_t lastPulse = 0;

  if (!enable) {
    if (init) ledcWrite(BUZZER_PIN, 0);
    return;
  }

  if (!init) {
    ledcAttach(BUZZER_PIN, CARRIER_HZ, PWM_RES);
    ledcWrite(BUZZER_PIN, 0);
    lastStep = millis();
    lastYelp = millis();
    init = true;
  }

  uint32_t now = millis();

  // ---------- Yelp timing ----------
  if (!yelpActive && (now - lastYelp >= YELP_INTERVAL_MS)) {
    yelpActive = true;
    lastYelp = now;
    lastPulse = now;
    yelpOn = true;
  }

  if (yelpActive && (now - lastYelp >= YELP_DURATION_MS)) {
    yelpActive = false;
    ledcWrite(BUZZER_PIN, duty);  // resume wail level
  }

  // ---------- Yelp behaviour ----------
  if (yelpActive) {
    if (now - lastPulse >= YELP_PULSE_MS) {
      lastPulse = now;
      yelpOn = !yelpOn;
      ledcWrite(BUZZER_PIN, yelpOn ? DUTY_MAX : 0);
    }
    return; // yelp overrides wail
  }

  // ---------- Wail behaviour ----------
  if (now - lastStep >= STEP_MS) {
    lastStep = now;

    int next = (int)duty + dir * STEP;
    if (next >= DUTY_MAX) { next = DUTY_MAX; dir = -1; }
    if (next <= DUTY_MIN) { next = DUTY_MIN; dir = +1; }
    duty = (uint16_t)next;

    ledcWrite(BUZZER_PIN, duty);
  }
} 
