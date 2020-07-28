//ch0
void ch0_rising_interrupt() {
  ch_0_rising = micros();
  attachInterrupt(1, 6, 6, ch0_falling_interrupt, FALLING);
}

void ch0_falling_interrupt() {
  ch0_duty_cycle = micros() - ch_0_rising;
  attachInterrupt(1, 6, 6, ch0_rising_interrupt, RISING);
}

//ch1
void ch1_rising_interrupt() {
  ch_1_rising = micros();
  attachInterrupt(1, 7, 7, ch1_falling_interrupt, FALLING);
}

void ch1_falling_interrupt() {
  ch1_duty_cycle = micros() - ch_1_rising;
  attachInterrupt(1, 7, 7, ch1_rising_interrupt, RISING);
}

//ch2
void ch2_rising_interrupt() {
  ch_2_rising = micros();
  attachInterrupt(1, 10, 10, ch2_falling_interrupt, FALLING);
}

void ch2_falling_interrupt() {
  ch2_duty_cycle = micros() - ch_2_rising;
  attachInterrupt(1, 10, 10, ch2_rising_interrupt, RISING);
}

//ch3
void ch3_rising_interrupt() {
  ch_3_rising = micros();
  attachInterrupt(1, 11, 11, ch3_falling_interrupt, FALLING);
}

void ch3_falling_interrupt() {
  ch3_duty_cycle = micros() - ch_3_rising;
  attachInterrupt(1, 11, 11, ch3_rising_interrupt, RISING);
}

//ch4
void ch4_rising_interrupt() {
  ch_4_rising = micros();
  attachInterrupt(0, 20, 4, ch4_falling_interrupt, FALLING);
}

void ch4_falling_interrupt() {
  ch4_duty_cycle = micros() - ch_4_rising;
  attachInterrupt(0, 20, 4, ch4_rising_interrupt, RISING);
}

//ch5
void ch5_rising_interrupt() {
  ch_5_rising = micros();
  attachInterrupt(1, 5, 5, ch5_falling_interrupt, FALLING);
}

void ch5_falling_interrupt() {
  ch5_duty_cycle = micros() - ch_5_rising;
  attachInterrupt(1, 5, 5, ch5_rising_interrupt, RISING);
}

//ch6
void ch6_rising_interrupt() {
  ch_6_rising = micros();
  attachInterrupt(0, 16, 0, ch6_falling_interrupt, FALLING);
}

void ch6_falling_interrupt() {
  ch6_duty_cycle = micros() - ch_6_rising;
  attachInterrupt(0, 16, 0, ch6_rising_interrupt, RISING);
}

//ch7
void ch7_rising_interrupt() {
  ch_7_rising = micros();
  attachInterrupt(1, 2, 2, ch7_falling_interrupt, FALLING);
}

void ch7_falling_interrupt() {
  ch7_duty_cycle = micros() - ch_7_rising;
  attachInterrupt(1, 2, 2, ch7_rising_interrupt, RISING);
}

// Interrupts for motor encoders
void right_wheel_encoder_interrupt() {
  if (digitalRead(right_wheel_encoder_b_pin)) {
    right_wheel_pulses++;
  } else {
    right_wheel_pulses--;
  }
}

void left_wheel_encoder_interrupt() {
  if (digitalRead(left_wheel_encoder_b_pin)) {
    left_wheel_pulses++;
  } else {
    left_wheel_pulses--;
  }
}
