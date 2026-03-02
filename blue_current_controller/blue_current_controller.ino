/*
 Fabry-Perot Scan Asynchronous Data Acquisition + Feedback Control and Communication 
*/
#include <math.h>
#include <ctype.h>
#include <Arduino.h>


//GPIO Pin IDs
#define ANALOG_PIN A0 // Fabry Perot input
#define TRIGGER_PIN 13 // Trigger input
#define MOD_DAC DAC0 // Current modulation out 

#define DIGILOCK_STATUS 30 // Digital lock status
#define MOD_ENABLE 28 // Feedback enable
#define MOD_ACTIVE 26 // Feedback active status
#define MOD_FAILURE 24 // Feedback failure status

#define BUSY_FLAG 3 // Busy flag
#define LOOP_TOGGLE 2 //External monitoring pin

// Other constants
#define N 500              // Samples per scan
#define HIGH_THRESH 2000
#define NUM_INIT_PEAKS 400 // 5s average 40 Hz * 2 peaks per scan
#define MAX_INIT_SCANS 700 // tries
#define PEAKS_PER_SCAN 2 
#define RUNNING_BUFFER_SIZE 200 // at 40Hz and 2 samples per scan this is 2.5 seconds of data

// state variables with corresponding status pins
bool digilock_status_flag = false;
bool mod_enable_flag = false;
bool mod_failure_flag = false;
bool mod_active_flag = false;

bool busy_flag = false;
bool loop_toggle = false;


// current data buffers and vars
static volatile uint16_t pdc_buf0[N]; // just for PDC writing
static volatile uint16_t pdc_buf1[N]; // alternating buffers so never busy waiting for PDC
static volatile uint16_t pdc_buf2[N]; // necessary to avoid serial delay failure case
volatile int pdc_last_buf = 1; // which buffer was last filled by PDC
volatile bool fresh_data = false; // flag set by ISR when new data ready

volatile uint16_t buffer[N]; // for main loop usage

// Dont think these need to be volatile
volatile uint16_t tempPeaks[PEAKS_PER_SCAN]; // need this one
volatile uint16_t tempPeakPos[PEAKS_PER_SCAN]; // Just 4 debug
volatile int tempPeakNo = 0;


// other state variables
volatile int adcIndex = 0;
volatile int delayus = 200;
bool initialized = false;


// initialization variables
int foundInitPeakNo = 0;
uint16_t initPeaks[NUM_INIT_PEAKS];
uint16_t initPeakPos[NUM_INIT_PEAKS]; // Just 4 debug


// feedback parameters
int shortMem = 20; // no traces between each feedback iteration, this is half a second
int countdown = 0; // gets set and decremented
float long_mem_n_stdev = 1.0; // no stdevs below init height to trigger peak lost
float short_mem_n_stdev = 1.0; // no stdevs below init height to trigger peak lost
int bump_count = 0;
int max_bumps = 100;
int min_adclvl = 10; //FIX l8r // minimum acceptable adc level before stopping feedback
int adclvl = 2048; // current adc level
int adc_step = 20; // step size for adc level bumps


// ---------- Peak statistics structure ----------
struct PeakStats {
 float meanHeight;
 float stdHeight;
};


// ---------- Global stats ----------
PeakStats initPeakStats;
float initHeight;
float initStd;

// ------------Running Buffer works as integrator for feedback control -------------
struct RunningBuffer {
  uint16_t* data;    // Pointer to dynamically allocated buffer
  int size;          // Max number of samples
  int head;          // Next index to write
  int count;         // Current number of elements

  float mean;        // Running mean
  float M2;          // Sum of squared differences

  RunningBuffer(int n) : size(n), head(0), count(0), mean(0.0f), M2(0.0f) {
    data = new uint16_t[n];
  }

  ~RunningBuffer() {
    delete[] data;
  }

  void push(uint16_t value) {
    data[head] = value;
    head = (head + 1) % size;
    if (count < size) count++;

    // Recompute mean and M2
    mean = 0.0f;
    for (int i = 0; i < count; i++) mean += get(i);
    mean /= count;

    M2 = 0.0f;
    for (int i = 0; i < count; i++) {
      float delta = (float)get(i) - mean;
      M2 += delta * delta;
    }
  }
  uint16_t get(int i) const {
    if (i >= count) return 0;
    int index = (head - count + i + size) % size;
    return data[index];
  }
  uint16_t latest() const {
    if (count == 0) return 0;
    int idx = (head - 1 + size) % size;
    return data[idx];
  }
  float getMean() const {
    return mean;
  }
  float getStd() const {
    if (count < 2) return 0.0f;
    return sqrt(M2 / (count - 1));
  }
  float getShortMean(int n) const {
    if (n > count) n = count;
    float shortMean = 0.0f;
    for (int i = count - n; i < count; i++) {
      shortMean += get(i);
    }
    shortMean /= n;
    return shortMean;
  }
};


RunningBuffer runningBuffer(RUNNING_BUFFER_SIZE);




// ---------- SETUP and LOOP ----------

void setup() {
  // Use the Programming port (Serial) so the device matches /dev/ttyACM1
  Serial.begin(250000);
  pinModeSetup();
  scopeSetup();
  analogWriteResolution(12);
  analogWrite(MOD_DAC, 0); // set initial current mod output lvl
  fastDigWrite(LOOP_TOGGLE, false); // for profiling
}

void profilingHelper(uint32_t arduinoPin, int no_flash){
  fastDigWrite(arduinoPin, false); 
  for (int i = 0; i < no_flash; i++) {
    fastDigWrite(arduinoPin, true);
    delayMicroseconds(500);
    fastDigWrite(arduinoPin, false);
    delayMicroseconds(500);
  }
}

void loop() {

  if (fresh_data) { //check that parallel DAQ loop has updated
    profilingHelper(BUSY_FLAG, 1);
    acquireScan();
    profilingHelper(LOOP_TOGGLE, 2);
    findPeaks();
    for (int i = 0; i < tempPeakNo; i++) {
      runningBuffer.push(tempPeaks[i]); 
    }

    //status update and feedback
    if (statusWrapper()) {
      feedbackWrapper();
    }

    // fastDigWrite(LOOP_TOGGLE, false); // for profiling
  }
  
  // HANDLE SERIAL COMMANDS
  char cmd = '\0';
  long arg = 0;
  if (readCommandLetterNumber(cmd, arg)) {

    // process parsed command
    // if (cmd == 'R') {
    //   Serial.write((uint8_t*)buffer, N * sizeof(uint16_t));
    //   Serial.flush();
    if (cmd == 'R') {
      Serial.println("R_RECEIVED");            // debug ack (temporary)
      Serial.write((uint8_t*)buffer, N * sizeof(uint16_t));
      // for debugging:
      findPeaks();
      Serial.print("Peaks found: ");
      Serial.println(tempPeakNo);
      for (int i = 0; i < tempPeakNo; i++) {
        Serial.print("Peak "); Serial.print(i);
        Serial.print(": val="); Serial.print(tempPeaks[i]);
        Serial.print(" pos="); Serial.println(tempPeakPos[i]);
      }
      // end debugging
      Serial.flush();
    } else if (cmd == 'D') {
      delayus = (int)arg;
      Serial.print("Delay set to: ");
      Serial.println(delayus);
    } else if (cmd == 'I') {
      initialize_peak_vals_locations();
    } else if (cmd == 'S') {
      Serial.println("S_RECEIVED");            // debug ack (temporary)
      sendCurrentStats();
    }


  }
}



// ----------SETUP HELPER FUNCS ----------


void pinModeSetup(){
  pinMode(ANALOG_PIN, INPUT);
  pinMode(TRIGGER_PIN, INPUT);
  pinMode(DIGILOCK_STATUS, INPUT);
  pinMode(MOD_ENABLE, INPUT);
  pinMode(MOD_ACTIVE, OUTPUT);
  pinMode(MOD_FAILURE, OUTPUT);
  pinMode(BUSY_FLAG, OUTPUT);
  pinMode(LOOP_TOGGLE, OUTPUT);

  digitalWrite(MOD_ACTIVE, LOW);
  digitalWrite(MOD_FAILURE, LOW);
  digitalWrite(BUSY_FLAG, LOW);
  digitalWrite(LOOP_TOGGLE, LOW);
}

void scopeSetup() {
  // set up simple trigger interrupt
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), onTriggerRise, RISING); 

  // clks need to be enabled for config
  pmc_enable_periph_clk(ID_TC2);            // TC0 ch2 will be the trigger delay timer
  pmc_enable_periph_clk(ID_TC0);            // TC0 ch0 will be the ADC sampling timer
  pmc_enable_periph_clk(ID_ADC);            // enable ADC peripheral clock


  // setup timer0 ch2 for trigger delay
  TC_Configure(TC0, 2,                      // Use channel 2 of TC module 0
               TC_CMR_TCCLKS_TIMER_CLOCK1 | // Select clock 1-4 (MCK/2,8,32,128 or Slow Clock), MCK = 84MHz
               TC_CMR_WAVE |                // Waveform mode (not input mode)
               TC_CMR_WAVSEL_UP_RC |        // Count up to RC then reset
               TC_CMR_ACPC_TOGGLE |         // toggle TIOA on RC compare
               TC_CMR_CPCSTOP               // Counter stops on RC compare just in case high priority interrupt cuts off clean up behavior
              ); 
  uint32_t default_delay = 4200;                     // 100us at 84MHz/2            
  TC_SetRC(TC0, 2, default_delay);              // Set RC for 1ms at 84MHz/2/1000

  TC0->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;      // enable interrupt flag on counter reaching RC value
  NVIC_SetPriority(TC2_IRQn, 3);                // set priority of that^ interrupt. TC2_IRQn handles timer2 channel 0
  NVIC_EnableIRQ(TC2_IRQn);                     // enable interrupt using NVIC (will require handler function)


  // within the timer0 block, route the output TIOA from ch2 to the trigger input of ch0 through the XC0 channel
  TC0->TC_BMR &= ~TC_BMR_TC0XC0S_Msk;
  TC0->TC_BMR |= TC_BMR_TC0XC0S_TIOA2; // route TIOA2 to XC0


  // setup timer0 ch0 for ADC sampling timing
  TC_Configure(TC0, 0,                      
               TC_CMR_TCCLKS_TIMER_CLOCK2 |  // set to same as ADC clock for easier timing
               TC_CMR_WAVE |                
               TC_CMR_WAVSEL_UP_RC |  
               TC_CMR_EEVTEDG_EDGE |          // trigger count start on either edge
               TC_CMR_EEVT_XC0 |              // trigger event from TIOA through the XC0 channel (timer0 ch1 toggles TIOA)
               TC_CMR_ENETRG |                // enable external event trigger
               TC_CMR_ACPC_TOGGLE             // clock TIOA0 
              );

  uint32_t default_samp = 52;                     // TODO: THIS IS HALF PERIOD for conversions REMEMBER
  TC_SetRC(TC0, 0, default_samp);                  // TODO: ALSO remember pos edge ADC so need to make sure this always starts toggling high for consistency

  // TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;      //TODO: removed interrupts here rememeber 
  // NVIC_SetPriority(TC0_IRQn, 3);                
  // NVIC_EnableIRQ(TC0_IRQn);
  

  // setup ADC to trigger on timer0 ch0 TIOA output
  ADC->ADC_CR = ADC_CR_SWRST; // reset ADC
  ADC->ADC_CHDR = 0xFFFFFFFF; // disable all channels
  ADC->ADC_CHER = ADC_CHER_CH7; // enable just channel 7 (A0)

  ADC->ADC_MR = (ADC->ADC_MR & ~ADC_MR_TRGSEL_Msk) | ADC_MR_TRGSEL_ADC_TRIG1; //trig1 is TIOA0 from TC0 ch0, set as trigger
  ADC->ADC_MR |= ADC_MR_TRGEN; // enable hardware trigger
  ADC->ADC_MR &= ~ADC_MR_SLEEP_SLEEP; // disable sleep mode
  ADC->ADC_MR = (ADC->ADC_MR & ~ADC_MR_STARTUP_Msk) | ADC_MR_STARTUP_SUT0;   // set startup time to 0
  ADC->ADC_MR &= ~ADC_MR_FREERUN_ON; // disable free run mode
  ADC->ADC_MR &= 0xFFFFFFEF; // set 12 bit resolution
  ADC->ADC_MR &= 0xFFCFFFFF; // set settling time to 3 ADC clocks
  ADC->ADC_MR &= 0xF0FFFFFF; // Tracking Time = (TRACKTIM + 1) * ADCClock periods: set to 1 ADC clock 
  ADC->ADC_MR &= 0xCFFFFFFF; // Transfer Period = (TRANSFER * 2 + 3) ADCClock periods: set to 3 ADC clocks

  ADC->ADC_EMR &= ~ADC_EMR_TAG; // disable channel number tagging in data register

  // make sure to set the trigger period greater than 13x adc clocks
  ADC->ADC_MR |= ADC_MR_PRESCAL(3); // set prescaler (ADC clock = MCK/(2*(PRESCAL+1)) 10.5 MHz

  // ADC->ADC_IER = ADC_IER_EOC7;   // enable End Of Conversion interrupt on ch7
  ADC->ADC_IDR = ADC_IDR_EOC7;   // disable this interrupt if using PDC
  

  // Set up PDC for ADC
  ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS; // disable PDC transfers and recieves while setting up
  
  ADC->ADC_RPR = (uint32_t)pdc_buf0; // set receive pointer to allocated PDC buffer
  // ADC->ADC_RPR = (uint32_t)(uintptr_t)&pdc_buf[0]; // maybe try this instead^
  ADC->ADC_RCR = N;                 // set receive counter to correct buffer size

  // clear next-pointer (no chained buffer neccessary for this)
  ADC->ADC_RNPR = 0;
  ADC->ADC_RNCR = 0;

  ADC->ADC_IER = ADC_IER_RXBUFF; // enable interrupt when buffer is full
  NVIC_SetPriority(ADC_IRQn, 3);
  NVIC_EnableIRQ(ADC_IRQn); 

  ADC->ADC_PTCR = ADC_PTCR_RXTEN; // enable PDC receive transfers


}


// --------------SCOPE RELATED INTERRUPT SERVICE ROUTINES ----------------

// handler for scope trigger
void onTriggerRise() {
  // fastDigWrite(LOOP_TOGGLE, true); // for profiling
  TC_Start(TC0, 2); // start and enable the delay timer (Ch2)
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN;  // JUST enable for now
}

// handler for timer0 ch2 interrupt (just for profiling)
extern "C" void TC2_Handler(void) {
  uint32_t sr = TC0->TC_CHANNEL[2].TC_SR;     // reads status register which autoclears flags
  // fastDigWrite(LOOP_TOGGLE, false);         // for profiling
}


// handler for ADC PDC buffer full interrupt
extern "C" void ADC_Handler(void) {
  uint32_t sr = ADC->ADC_ISR; // clears status bits
  ADC->ADC_PTCR = ADC_PTCR_RXTDIS; // stop RX while processing / rearming
  TC_Stop(TC0, 0); // stop ADC timer, disable the clock

  // fastDigWrite(LOOP_TOGGLE, true); // for profiling

  (void)TC0->TC_CHANNEL[0].TC_SR; // clear timer status register DONT NEED?
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG; // reset the counter but dont start as it is disabled

  if (pdc_last_buf == 2) {          // if last_buffer was buf2, we just filled buf0, so record last_buffer as 0 and rearm pdc with buf1
    ADC->ADC_RPR = (uint32_t)(uintptr_t)pdc_buf1;
    pdc_last_buf = 0;

  }else if (pdc_last_buf == 1) {
    ADC->ADC_RPR = (uint32_t)(uintptr_t)pdc_buf0; // if last was 1, just filled 2, so set last_buf to 2 and rearm with 0
    pdc_last_buf = 2;
  } else if (pdc_last_buf == 0) {
    ADC->ADC_RPR = (uint32_t)(uintptr_t)pdc_buf2; // if last was 0, just filled 1, so set last_buf to 1 and rearm with 2
    pdc_last_buf = 1;
  }

  fresh_data = true; // signal main loop that new data is ready

  // Re-arm PDC for next capture
  ADC->ADC_RCR = N; 

  ADC->ADC_PTCR = ADC_PTCR_RXTEN; // restart PDC RX
  // fastDigWrite(LOOP_TOGGLE, false); // for profiling

  
}


// ------------LOOP HELPER FUNCS -------------

void fastDigWrite(uint32_t arduinoPin, bool val) {
  Pio* port = g_APinDescription[arduinoPin].pPort;
  uint32_t mask = g_APinDescription[arduinoPin].ulPin;
  if (val) port->PIO_SODR = mask;
  else      port->PIO_CODR = mask;
}

bool fastDigRead(uint32_t arduinoPin) {
  Pio* port = g_APinDescription[arduinoPin].pPort;
  uint32_t mask = g_APinDescription[arduinoPin].ulPin;
  return (port->PIO_PDSR & mask) != 0;
}

// ---------- Scan acquisition ----------
void acquireScan() {
  // fastDigWrite(LOOP_TOGGLE, true); // for profiling
  fresh_data = false;
  if (pdc_last_buf == 2) {
    // last filled buffer was buf2, so copy buf2
    memcpy((void*)buffer, (const void*)pdc_buf2, N * sizeof(uint16_t)); // copies pdc buffer to main loop buffer for processing
  } else if (pdc_last_buf == 1) {
    memcpy((void*)buffer, (const void*)pdc_buf1, N * sizeof(uint16_t));
  } else if (pdc_last_buf == 0) {
    memcpy((void*)buffer, (const void*)pdc_buf0, N * sizeof(uint16_t));
  }
  // fastDigWrite(LOOP_TOGGLE, false); // for profiling
}

// ---------- Local max detection ----------
bool isLocalMax(int i, int window) {
 uint16_t val = buffer[i];
 for (int j = -window; j <= window; j++) {
   if (j == 0) continue;
   if (buffer[i + j] > val) return false;
 }
 return true;
}


// ---------- Find peaks in current scan ----------
void findPeaks() {
 int window = 5;
 int f = 0;
 tempPeakNo = 0;
 for (int i = window; i < N - window; i++) {
   uint16_t val = buffer[i];
   if (val > HIGH_THRESH && isLocalMax(i, window)) {
     if (f < PEAKS_PER_SCAN) {
       tempPeakPos[f] = i; // Just 4 debug
       tempPeaks[f++] = val; // need this one
       tempPeakNo++;
     }
   }
 }
}


// ---------- Check if peak lost ----------
bool isPeakLost(float maxVal, float meanHeight, float stdHeight) {
 if (maxVal < (meanHeight - stdHeight)) return true;
 return false;
}


bool statusWrapper() { // takes about 9 microseconds with slow read/write
  // fastDigWrite(LOOP_TOGGLE, true);   // speed profiling
  digilock_status_flag = fastDigRead(DIGILOCK_STATUS);
  mod_enable_flag = fastDigRead(MOD_ENABLE);

  fastDigWrite(MOD_ACTIVE, mod_active_flag); // these two get set in the feedback wrapper so don't need to change here, just output 
  fastDigWrite(MOD_FAILURE, mod_failure_flag);

  // fastDigWrite(LOOP_TOGGLE, false); // profiling

  // make sure all checks passed
  if (mod_failure_flag) {
    profilingHelper(LOOP_TOGGLE, 20);
  }
  return digilock_status_flag && mod_enable_flag && !mod_failure_flag && initialized;
}


void feedbackWrapper() {
  float long_term_mean = runningBuffer.getMean();
  float short_term_mean = runningBuffer.getShortMean(shortMem * 2); // 2 peaks per trace

  if (!mod_active_flag && isPeakLost(long_term_mean, initHeight, long_mem_n_stdev * initStd)) {
    mod_active_flag = true; // activate feedback with some hysteresis
    analogWrite(MOD_DAC, adclvl); // for profiling FIX l8r
    countdown = 0;
  }

  if (mod_active_flag && countdown == 0) { // time to check short term memory
    profilingHelper(LOOP_TOGGLE, 5);
    countdown = shortMem; // reset countdown

    if (bump_count > max_bumps || adclvl <= (min_adclvl + adc_step)) {
      mod_failure_flag = true; // fail if too many bumps or out of range, gets us permanently out of feedback loop
      mod_active_flag = false;  

    } else if (isPeakLost(short_term_mean, initHeight, short_mem_n_stdev * initStd)) {
      adclvl -= adc_step;   // bump current modulation down
      if (adclvl < 0) {
        adclvl = 0;
      }
      analogWrite(MOD_DAC, adclvl);
      profilingHelper(LOOP_TOGGLE, 10);
      bump_count++;
    } else {                // short term peak height is above stdev
      if (isPeakLost(long_term_mean, initHeight, long_mem_n_stdev * initStd)) {
        countdown = shortMem; // short term is good, wait for long term and keep checking. May need to add some kind of integral gain behavior here l8r
      } else {
        mod_active_flag = false; // disable feedback if long term memory is good
        bump_count = 0; 
      }
    }
  } else {
    countdown--; // fill up short term memory
  }
}



// ---------- Peak stats ----------
PeakStats computePeakStats(uint16_t *heights, int count) {
 PeakStats s = {0, 0};
 if (count <= 0) return s;
 float sumH = 0;
 for (int i = 0; i < count; i++) sumH += heights[i];
 s.meanHeight = sumH / count;
 float varH = 0;
 for (int i = 0; i < count; i++) varH += pow(heights[i] - s.meanHeight, 2);
 if (count > 1) s.stdHeight = sqrt(varH / (count - 1));
 return s;
}


// ---------- Initialization ----------
void initialize_peak_vals_locations() {
 foundInitPeakNo = 0;
 int scanCount = 0;

 while (scanCount < MAX_INIT_SCANS && foundInitPeakNo < NUM_INIT_PEAKS) {
   while (!fresh_data) {} // wait for new data
   acquireScan();
   findPeaks();
   scanCount++;
   for (int i = 0; i < tempPeakNo; i++) {
      if (foundInitPeakNo < NUM_INIT_PEAKS) {
        initPeakPos[foundInitPeakNo] = tempPeakPos[i]; // Just 4 debug
        initPeaks[foundInitPeakNo++] = tempPeaks[i]; // do need this one
      }
   }
 }

 initPeakStats = computePeakStats(initPeaks, foundInitPeakNo);
 initHeight = initPeakStats.meanHeight;
 initStd = initPeakStats.stdHeight;

 initialized = true;
 
//  Serial.println("[START] Initalization");
//  sendPeaks();
//  Serial.println("[END] Peaks");
 Serial.println("[START] Stats");
 sendInitStats(initPeakStats);
 Serial.println("[END] Stats");
}


void sendPeaks() {
  Serial.println("BEGIN_peaks");

  // Send peak info
  for (int i = 0; i < NUM_INIT_PEAKS; i++) {
    if (initPeaks[i]>0){
      Serial.print("val="); Serial.print(initPeaks[i]);
      Serial.print(" pos="); Serial.println(initPeakPos[i]); // Just 4 debug
    }
  }
  Serial.println("END_peaks");
}


void sendInitStats(PeakStats initPeakStats) {
  Serial.println("BEGIN_stats");

  Serial.print("meanHeight=");
  Serial.print(initPeakStats.meanHeight, 2);
  Serial.print(" stdHeight=");
  Serial.println(initPeakStats.stdHeight, 2);

  Serial.println("END_stats");
}

void sendCurrentStats() {
  Serial.println("BEGIN_currentStats");
  Serial.print("currentMean=");
  Serial.print(runningBuffer.getMean(), 2);
  Serial.print(" currentStd=");
  Serial.println(runningBuffer.getStd(), 2);
  Serial.println("END_currentStats");
}


// command format will be "LETTERS <NUMBER ARG>" and they will be saved to passed vars"
bool readCommandLetterNumber(char &cmdChar, long &value) {
  static char buf[64];
  static uint8_t len = 0;
  while (Serial.available()) {
    int c = Serial.read();
    if (c <= 0) break;
    if (c == '\r') continue;
    if (c == '\n') {
      if (len == 0) { len = 0; return false; }
      buf[len] = '\0';
      // parse: skip leading whitespace, first char = command
      char *p = buf;
      while (*p && isspace((unsigned char)*p)) p++;
      cmdChar = *p ? *p++ : '\0';
      // skip whitespace, parse number (if any)
      while (*p && isspace((unsigned char)*p)) p++;
      value = strtol(p, NULL, 10);
      len = 0;
      return true;
    }
    if (len < sizeof(buf) - 1) buf[len++] = (char)c;
    else len = 0; // overflow: reset buffer
  }
  return false;
}

