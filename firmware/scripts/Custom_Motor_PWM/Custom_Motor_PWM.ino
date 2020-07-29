//This was used for ForceTronics YouTube tutorial on generating PWM signals with SAMD21 based Arduinos
//This code is public domain and can be used by anyone at their own risk
//Some of this code was leveraged from MartinL on Arduino Forum http://forum.arduino.cc/index.php?topic=346731.5;wap2

//sets the period of the PWM signal, PWM period = wPer / gen clock rate 
volatile unsigned char wPer = 255;
//This variable is to generate the duty cycle of the PWM signal 0.5 --> 50%
volatile float pWMDC = .5;
//selects the gen clock for setting the waveform generator clock or sample rate
const unsigned char gClock = 4;
//sets the divide factor for the gen clk, 48MHz / 3 = 16MHz
const unsigned char dFactor = 3;

void setup() 
{ 
  pinMode(13, OUTPUT);
  analogReadResolution(8); //set the ADC resolution to match the PWM max resolution (0 to 255)
  
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(dFactor) |          // Divide the main clock down by some factor to get generic clock
                    GCLK_GENDIV_ID(gClock);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(gClock);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for the digital pin. Note commented out line is pin D7, other is D3
 // PORT->Group[g_APinDescription[7].ulPort].PINCFG[g_APinDescription[7].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[3].ulPort].PINCFG[g_APinDescription[3].ulPin].bit.PMUXEN = 1;
  
   //Connect the TCC0 timer to digital output - port pins are paired odd PMUO and even PMUXE (note D7 is commented out and D3 is not)
  // PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F; 
  PORT->Group[g_APinDescription[4].ulPort].PMUX[g_APinDescription[4].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  //Set for Single slope PWM operation: timers or counters count up to TOP value and then repeat
  REG_TCC1_WAVE |= TCC_WAVE_WAVEGEN_NPWM;       // Reverse the output polarity on all TCC0 outputs
                   //TCC_WAVE_POL(0xF)      //this line inverts the output waveform
                   //TCC_WAVE_WAVEGEN_DSBOTH;    // Setup dual slope PWM on TCC0
  while (TCC1->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation: 
  REG_TCC1_PER = wPer;         // This sets the rate or frequency of PWM signal. 
  while (TCC1->SYNCBUSY.bit.PER);                // Wait for synchronization
  
  // Set the PWM signal to output 50% duty cycle initially (0.5 x 255)
  REG_TCC1_CC1 = pWMDC*wPer;        
  while (TCC1->SYNCBUSY.bit.CC1);                // Wait for synchronization

  //enable interrupts
  REG_TCC1_INTENSET = TCC_INTENSET_OVF; //Set up interrupt at TOP of each PWM cycle
  enable_interrupts(); //enable in NVIC
  
  // Set prescaler and enable the outputs
  REG_TCC1_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

void loop() { 

  //Put main code here
 }

//This function sets the interrupts priority to highest and then enables the PWM interrupt
void enable_interrupts() {
  NVIC_SetPriority(TCC1_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority
  NVIC_EnableIRQ(TCC1_IRQn);
}

//This ISR is called at the end or TOP of each PWM cycle
void TCC1_Handler() {
    REG_TCC1_PER = analogRead(A1); //Get period from A1
    while (TCC1->SYNCBUSY.bit.PER);
    REG_TCC1_CC1 = (analogRead(A0)/255.0)*analogRead(A1); //calculate PWM using A0 reading and A1 current state
    while (TCC1->SYNCBUSY.bit.CC1);
    REG_TCC0_INTFLAG = TC_INTFLAG_OVF; //Need to reset interrupt
}
