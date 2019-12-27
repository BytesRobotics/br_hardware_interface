//ch1
void ch1_rising_interrupt(){
  ch_1_rising = micros();
  attachInterrupt(digitalPinToInterrupt(front_dist_pin), ch1_falling_interrupt, FALLING);
}

void ch1_falling_interrupt(){
  ch1_duty_cycle = micros()-ch_1_rising;
  attachInterrupt(digitalPinToInterrupt(front_dist_pin), ch1_rising_interrupt, RISING);
}

//ch2
void ch2_rising_interrupt(){
  ch_2_rising = micros();
  attachInterrupt(digitalPinToInterrupt(left_dist_pin), ch2_falling_interrupt, FALLING);
}

void ch2_falling_interrupt(){
  ch2_duty_cycle = micros()-ch_2_rising;
  attachInterrupt(digitalPinToInterrupt(left_dist_pin), ch2_rising_interrupt, RISING);
}

//ch3
void ch3_rising_interrupt(){
  ch_3_rising = micros();
  attachInterrupt(digitalPinToInterrupt(right_dist_pin), ch3_falling_interrupt, FALLING);
}

void ch3_falling_interrupt(){
  ch3_duty_cycle = micros()-ch_3_rising;
  attachInterrupt(digitalPinToInterrupt(right_dist_pin), ch3_rising_interrupt, RISING);
}

//ch4
void ch4_rising_interrupt(){
  ch_4_rising = micros();
  attachInterrupt(digitalPinToInterrupt(rear_dist_pin), ch4_falling_interrupt, FALLING);
}

void ch4_falling_interrupt(){
  ch4_duty_cycle = micros()-ch_4_rising;
  attachInterrupt(digitalPinToInterrupt(rear_dist_pin), ch4_rising_interrupt, RISING);
}

//ch5
void ch5_rising_interrupt(){
  ch_5_rising = micros();
  attachInterrupt(digitalPinToInterrupt(bottom_dist_pin), ch5_falling_interrupt, FALLING);
}

void ch5_falling_interrupt(){
  ch5_duty_cycle = micros()-ch_5_rising;
  attachInterrupt(digitalPinToInterrupt(bottom_dist_pin), ch5_rising_interrupt, RISING);
}

// Interrupts for motor encoders
void right_wheel_encoder_interrupt(){
  if(digitalRead(right_wheel_encoder_b_pin)){
    right_wheel_pulses++;
  } else {
    right_wheel_pulses--;
  }
}

void left_wheel_encoder_interrupt(){
    if(digitalRead(left_wheel_encoder_b_pin)){
    left_wheel_pulses++;
  } else {
    left_wheel_pulses--;
  }
}
