#define DIGILOCK_STATUS 30 // Digital lock status
#define MOD_ENABLE 28 // Feedback enable
#define MOD_ACTIVE 26 // Feedback active status
#define MOD_FAILURE 24 // Feedback failure status
#define PEAKS_LOST 32 // for logging
bool digilock_status_flag = true;
bool mod_enable_flag = true;
bool mod_failure_flag = false;
bool mod_active_flag = false;
bool peaks_lost_flag = false;

bool init_done = false;


void setup() {
  pinMode(DIGILOCK_STATUS, INPUT);
  pinMode(MOD_ENABLE, INPUT);
  pinMode(MOD_ACTIVE, OUTPUT);
  pinMode(MOD_FAILURE, OUTPUT);
  pinMode(PEAKS_LOST, OUTPUT);
  digitalWrite(MOD_ACTIVE, mod_active_flag);
  digitalWrite(MOD_FAILURE, mod_failure_flag);

  SerialUSB.begin(250000);  // Programming USB (now to Pi)
  while (!SerialUSB);
  SerialUSB.println("CONNECTED");
}

void loop() {
  if (SerialUSB.available()) {
    String cmd = SerialUSB.readStringUntil('\n');
    mod_enable_flag = digitalRead(MOD_ENABLE);
    digilock_status_flag = digitalRead(DIGILOCK_STATUS);

    SerialUSB.print("Echo: ");
    SerialUSB.println(cmd);

    SerialUSB.print("flags: ");
    SerialUSB.print(mod_enable_flag);
    SerialUSB.print(", ");
    SerialUSB.println(digilock_status_flag);
    init_done = true;
  }
  if (init_done) {
    while (!digitalRead(MOD_ENABLE)) {}
    digitalWrite(MOD_ACTIVE, true);

    delay(10000);
    while (true) {
      digitalWrite(MOD_ACTIVE, mod_active_flag);
      digitalWrite(PEAKS_LOST, peaks_lost_flag);
      mod_active_flag = !mod_active_flag;
      peaks_lost_flag = !peaks_lost_flag;
      delay(1000);
    }
  }
}