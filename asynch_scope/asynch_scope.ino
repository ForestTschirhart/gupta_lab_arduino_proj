/*
 FABRY-PEROT SCAN ASYNCHRONOUS DATA ACQUISITION ONLY
  Trimmed down from full feedback controller version for clarity
  in case someone want to use as reference.
  Arduino Due Version
*/
#include <math.h>
#include <ctype.h>
#include <Arduino.h> // this is where a lot of the headers for readability come from


//GPIO Pin IDs
#define ANALOG_PIN A0 // Signal input
#define TRIGGER_PIN 13 // Trigger input

#define BUSY_FLAG 8 // Busy flag
#define LOOP_TOGGLE 2 //External monitoring pin


// Other constants
#define N 500              // Samples per scan
#define HIGH_THRESH 2000   // Threshold for peak detection
#define PEAKS_PER_SCAN 2   // Number of peaks to detect per scan

// debugging variables
bool loop_toggle = false;


// current data buffers and vars
static volatile uint16_t pdc_buf0[N]; // just for PDC to write to
static volatile uint16_t pdc_buf1[N]; // alternating buffers so never busy waiting for PDC

volatile int pdc_last_buf = 1; // which buffer was last filled by PDC
volatile bool fresh_data = false; // flag set by ISR when new data ready

volatile uint16_t buffer[N]; // for main loop usage

// other variables for trace processing
volatile uint16_t tempPeaks[PEAKS_PER_SCAN];
volatile uint16_t tempPeakPos[PEAKS_PER_SCAN];
volatile int tempPeakNo = 0;



// ---------- SETUP and LOOP ----------

void setup() {
  Serial.begin(250000);   // Use the Programming port for debugging (Serial) so the device matches /dev/ttyACM1
  pinModeSetup();
  scopeSetup();
}

void loop() {
  // fastDigWrite(LOOP_TOGGLE, true); // for profiling
  // fastDigWrite(LOOP_TOGGLE, false); // for profiling
  if (fresh_data) { // check if new data ready
    fastDigWrite(LOOP_TOGGLE, true); // for profiling
    acquireScan();      // handles buffer copying and flags
    findPeaks();        // some main loop CPU processing just to show asynchronicity

    fastDigWrite(LOOP_TOGGLE, false); // for profiling
  }
}



// ----------SETUP HELPER FUNCS ----------

void pinModeSetup(){
  pinMode(ANALOG_PIN, INPUT);
  pinMode(TRIGGER_PIN, INPUT);
  pinMode(BUSY_FLAG, OUTPUT);
  pinMode(LOOP_TOGGLE, OUTPUT);

  digitalWrite(BUSY_FLAG, LOW);
  digitalWrite(LOOP_TOGGLE, LOW);
}


// This is where most of the complexity is
void scopeSetup() {
  // clks need to be enabled for config
  pmc_enable_periph_clk(ID_TC2);            // TC0 ch2 will be the trigger delay timer (Ch1 seems to be involved with one of the ADCs)
  pmc_enable_periph_clk(ID_TC0);            // TC0 ch0 will be the ADC sampling timer  (Think you need to use this module and channel)
  pmc_enable_periph_clk(ID_ADC);            // enable ADC peripheral clock

  
  // ------------ set up simple trigger interrupt ------------------
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), onTriggerRise, RISING); 



  // -------------- setup timer0 ch2 for trigger delay --------------
  TC_Configure(TC0, 2,                      // Use channel 2 of TC module 0
               TC_CMR_TCCLKS_TIMER_CLOCK1 | // Select clock 1-4 (MCK/2,8,32,128 or Slow Clock), MCK = 84MHz
               TC_CMR_WAVE |                // Waveform mode (not input mode)
               TC_CMR_WAVSEL_UP_RC |        // Count up to RC value then reset
               TC_CMR_ACPC_TOGGLE |         // toggle TIOA (2) on RC compare
               TC_CMR_CPCSTOP               // Counter stops on RC compare just in case high priority interrupt cuts off clean up behavior
              ); 
  uint32_t default_delay = 4200;                // 100us at 84MHz/2            
  TC_SetRC(TC0, 2, default_delay);              // Set RC


  // //UNCOMMENT TO ENABLE INTERRUPT ON DELAY TIMER FINISH
  // TC0->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;      // enable interrupt flag on counter reaching RC value
  // NVIC_SetPriority(TC2_IRQn, 3);                // set priority of that^ interrupt. TC2_IRQn handles timer2 channel 0
  // NVIC_EnableIRQ(TC2_IRQn);                     // enable interrupt using NVIC (will require handler function)

  // ----------- link timer0 ch2 to timer0 ch0 start trigger -----------

  // within the timer0 block, route the output TIOA from ch2 to the trigger input of ch0 through the XC0 channel
  TC0->TC_BMR &= ~TC_BMR_TC0XC0S_Msk;
  TC0->TC_BMR |= TC_BMR_TC0XC0S_TIOA2; // route TIOA2 to XC0


  // ----------------------- setup timer0 ch0 for ADC sample timing ------------------------
  TC_Configure(TC0, 0,                      
               TC_CMR_TCCLKS_TIMER_CLOCK2 |   // set to same as ADC clock for easier timing calculations
               TC_CMR_WAVE |                
               TC_CMR_WAVSEL_UP_RC |  
               TC_CMR_EEVTEDG_EDGE |          // Trigger count start on either edge of delay timer output
               TC_CMR_EEVT_XC0 |              // Trigger event from TIOA through the XC0 channel (timer0 ch2 toggles TIOA)
               TC_CMR_ENETRG |                // Enable external event trigger
               TC_CMR_ACPC_TOGGLE             // Toggle TIOA0 on RC compare to drive ADC triggering
              );

  uint32_t default_samp = 52;                     // This is HALF PERIOD for conversions, ADC uses rising edge
  TC_SetRC(TC0, 0, default_samp);                 // In ISR make sure this always gives rising edge first after delay timer
 


  // ---------------------- setup ADC to trigger on timer0 ch0 TIOA output ------------------
  // Adjust ADC Config alongside Timer0 ch0 settings to get desired sample rate, can be finnicky

  ADC->ADC_CR = ADC_CR_SWRST; // reset ADC
  ADC->ADC_CHDR = 0xFFFFFFFF; // disable all channels
  ADC->ADC_CHER = ADC_CHER_CH7; // enable just channel 7 (Pin A0)

  ADC->ADC_MR = (ADC->ADC_MR & ~ADC_MR_TRGSEL_Msk) | ADC_MR_TRGSEL_ADC_TRIG1; // trig1 is TIOA0 from TC0 ch0, set as trigger
  ADC->ADC_MR |= ADC_MR_TRGEN;                      // enable hardware trigger
  ADC->ADC_MR &= ~ADC_MR_SLEEP_SLEEP;               // disable sleep mode
  ADC->ADC_MR = (ADC->ADC_MR & ~ADC_MR_STARTUP_Msk) | ADC_MR_STARTUP_SUT0;   // set startup time to 0 ADCclk cycle
  ADC->ADC_MR &= ~ADC_MR_FREERUN_ON;                                         // disable free run mode
  ADC->ADC_MR &= 0xFFFFFFEF;                       // set 12 bit resolution
  ADC->ADC_MR &= 0xFFCFFFFF;                       // set settling time to 3 ADC clocks (min)
  ADC->ADC_MR &= 0xF0FFFFFF;                       // Tracking Time = (TRACKTIM + 1) * ADCClock periods: set to 1 ADC clock 
  ADC->ADC_MR &= 0xCFFFFFFF;                       // Transfer Period = (TRANSFER * 2 + 3) ADCClock periods: set to 3 ADC clocks

  // make sure to set the trigger period greater than ~13x adc clocks (At least 20 to be safe IMO)
  ADC->ADC_MR |= ADC_MR_PRESCAL(3); // set prescaler (ADC clock = MCK/(2*(PRESCAL+1)) 10.5 MHz



  // ------------------------ Set up PDC for ADC -----------------------
  ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS; // disable PDC transfers and recieves while setting up


  ADC->ADC_EMR &= ~ADC_EMR_TAG;                    // disable channel number tagging in data register
  ADC->ADC_RPR = (uint32_t)pdc_buf0;               // set receive pointer to allocated PDC buffer (buff0 first but doesn't matter)
  ADC->ADC_RCR = N;                                // set receive counter to correct buffer size

  ADC->ADC_RNPR = 0;                               // clear next-pointer (no chained buffer neccessary for this)    
  ADC->ADC_RNCR = 0;

  ADC->ADC_IDR = ADC_IDR_EOC7;                     // disable individual ADC conversion done interrupts
  ADC->ADC_IER = ADC_IER_RXBUFF;                   // enable interrupt when PDC buffer is full
  NVIC_SetPriority(ADC_IRQn, 3);                   // set priority of ADC interrupt
  NVIC_EnableIRQ(ADC_IRQn);                        // enable ADC interrupt using NVIC handler framework

  ADC->ADC_PTCR = ADC_PTCR_RXTEN; // enable PDC receive transfers


}


// --------------SCOPE RELATED INTERRUPT SERVICE ROUTINES ----------------

// handler for scope trigger
void onTriggerRise() {
  fastDigWrite(LOOP_TOGGLE, true);     // for profiling
  TC_Start(TC0, 2);                    // start and enable the delay timer (Ch2)
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN;  // JUST enable for now, will start on autotrigger from delay timer
}


// // UNCOMMENT TO ENABLE INTERRUPT ON DELAY TIMER FINISH
// // handler for timer0 ch2 interrupt 
// extern "C" void TC2_Handler(void) {
//   uint32_t sr = TC0->TC_CHANNEL[2].TC_SR;     // reads status register which autoclears flags
//   fastDigWrite(LOOP_TOGGLE, false); // for profiling
// }


// handler for ADC PDC buffer full interrupt
extern "C" void ADC_Handler(void) {
  uint32_t sr = ADC->ADC_ISR;             // read clears status bits
  ADC->ADC_PTCR = ADC_PTCR_RXTDIS;        // stop RX while processing / rearming
  TC_Stop(TC0, 0);                        // Disable ADC timer

  fastDigWrite(LOOP_TOGGLE, true);        // for profiling

  (void)TC0->TC_CHANNEL[0].TC_SR;         // clear timer status register (Potentially don't need cause not a TC interrupt?)
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG; // reset the counter but dont start as it is disabled

  if (pdc_last_buf == 1) {                 // if "last buffer" was buf1 -> we just filled buf0 -> set last to 0 and rearm PDC with buf1
    ADC->ADC_RPR = (uint32_t)(uintptr_t)pdc_buf1;
    pdc_last_buf = 0;
  } else if (pdc_last_buf == 0) {          // else do the opposite
    ADC->ADC_RPR = (uint32_t)(uintptr_t)pdc_buf0;
    pdc_last_buf = 1;
  }

  fresh_data = true;                        // signal main loop that new data is ready

  ADC->ADC_RCR = N;                         // Re-arm PDC count for next capture

  ADC->ADC_PTCR = ADC_PTCR_RXTEN;           // restart PDC RX
  fastDigWrite(LOOP_TOGGLE, false);         // for profiling

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
  fresh_data = false;
  if (pdc_last_buf == 1) {
    // last filled buffer was buf1, so copy buf1
    memcpy((void*)buffer, (const void*)pdc_buf1, N * sizeof(uint16_t)); // copies appropriate pdc buffer to main loop buffer for processing
  } else if (pdc_last_buf == 0) {
    memcpy((void*)buffer, (const void*)pdc_buf0, N * sizeof(uint16_t));
  }
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
       tempPeakPos[f] = i;
       tempPeaks[f++] = val;
       tempPeakNo++;
     }
   }
 }
}

