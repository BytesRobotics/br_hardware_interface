#define CH0 A3
volatile unsigned long ch_0_rising, ch0_duty_cycle;
void ch0_rising_interrupt();
void ch0_falling_interrupt();

void setup() {
  SerialUSB.begin(9600);
  while(!SerialUSB);
  pinMode(CH0, INPUT);
  attachInterrupt(digitalPinToInterrupt(CH0), ch0_rising_interrupt, RISING);
  config_pwm();
  SerialUSB.println("Setup complete");
}

void loop() {
  SerialUSB.println("CH0: " + String(ch0_duty_cycle));
  //SerialUSB.println(ch0_duty_cycle));
}

//ch0
void ch0_rising_interrupt() {
  ch_0_rising = micros();
  attachInterrupt(digitalPinToInterrupt(CH0), ch0_falling_interrupt, FALLING);
}

void ch0_falling_interrupt() {
  ch0_duty_cycle = micros() - ch_0_rising;
  attachInterrupt(digitalPinToInterrupt(CH0), ch0_rising_interrupt, RISING);
}

void config_pwm() {

  //PWM resolution = frequency_gclock/N(TOP+1) N is clock divider which is 256 for us

  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(256) |          // Divide the 48MHz clock source by divisor 480: 48MHz/480=0.1MHz (same number as below)
                    GCLK_GENDIV_ID(4);              // Select Generic Clock (GCLK) 4

  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4

  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for the digital pin D7
  SerialUSB.print("Port ");
  SerialUSB.println(g_APinDescription[7].ulPort); // shoudl be 0
  SerialUSB.print("Pin ");
  SerialUSB.println(g_APinDescription[7].ulPin); //should be 21
  PORT->Group[0].PINCFG[21].bit.PMUXEN = 1;

  // Connect the TCC0 timer to digital output D7 - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  // g_APinDescription[6] = (0,20)
  PORT->Group[0].PMUX[20 >> 1].reg = PORT_PMUX_PMUXO_F;

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1

  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;    // Setup single slope PWM on TCC0

  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  REG_TCC0_PER = 9000;         // Set the frequency of the PWM on TCC0 to 10 (v)(4500 for 24ms)

  while (TCC0->SYNCBUSY.bit.PER);                // Wait for synchronization

  // Set the PWM signal to output 10us duty cycle
  REG_TCC0_CC3 = 2;         // TCC0 CC3 - on D7 (y)

  while (TCC0->SYNCBUSY.bit.CC3);                // Wait for synchronization

  // Divide the 48MHz signal by 256 giving 48MHz (5.33us) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV256 |    // Divide GCLK4 by 1 Available: (1, 2, 4, 8, 16, 64, 256, 1024)
                    TCC_CTRLA_ENABLE;               // Enable the TCC0 output

  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

}
