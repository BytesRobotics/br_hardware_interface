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
  attachInterrupt(digitalPinToInterrupt(front_left_dist_pin), ch2_falling_interrupt, FALLING);
}

void ch2_falling_interrupt(){
  ch2_duty_cycle = micros()-ch_2_rising;
  attachInterrupt(digitalPinToInterrupt(front_left_dist_pin), ch2_rising_interrupt, RISING);
}

//ch3
void ch3_rising_interrupt(){
  ch_3_rising = micros();
  attachInterrupt(digitalPinToInterrupt(front_right_dist_pin), ch3_falling_interrupt, FALLING);
}

void ch3_falling_interrupt(){
  ch3_duty_cycle = micros()-ch_3_rising;
  attachInterrupt(digitalPinToInterrupt(front_right_dist_pin), ch3_rising_interrupt, RISING);
}

//ch4
void ch4_rising_interrupt(){
  ch_4_rising = micros();
  attachInterrupt(digitalPinToInterrupt(front_bottom_dist_pin), ch4_falling_interrupt, FALLING);
}

void ch4_falling_interrupt(){
  ch4_duty_cycle = micros()-ch_4_rising;
  attachInterrupt(digitalPinToInterrupt(front_bottom_dist_pin), ch4_rising_interrupt, RISING);
}

//ch5
void ch5_rising_interrupt(){
  ch_5_rising = micros();
  attachInterrupt(digitalPinToInterrupt(rear_dist_pin), ch5_falling_interrupt, FALLING);
}

void ch5_falling_interrupt(){
  ch5_duty_cycle = micros()-ch_5_rising;
  attachInterrupt(digitalPinToInterrupt(rear_dist_pin), ch5_rising_interrupt, RISING);
}

//ch6
void ch6_rising_interrupt(){
  ch_6_rising = micros();
  attachInterrupt(digitalPinToInterrupt(rear_left_dist_pin), ch6_falling_interrupt, FALLING);
}

void ch6_falling_interrupt(){
  ch6_duty_cycle = micros()-ch_6_rising;
  attachInterrupt(digitalPinToInterrupt(rear_left_dist_pin), ch6_rising_interrupt, RISING);
}

//ch7
void ch7_rising_interrupt(){
  ch_7_rising = micros();
  attachInterrupt(digitalPinToInterrupt(rear_right_dist_pin), ch7_falling_interrupt, FALLING);
}

void ch7_falling_interrupt(){
  ch7_duty_cycle = micros()-ch_7_rising;
  attachInterrupt(digitalPinToInterrupt(rear_right_dist_pin), ch7_rising_interrupt, RISING);
}

//ch8
void ch8_rising_interrupt(){
  ch_8_rising = micros();
  attachInterrupt(digitalPinToInterrupt(rear_bottom_dist_pin), ch8_falling_interrupt, FALLING);
}

void ch8_falling_interrupt(){
  ch8_duty_cycle = micros()-ch_8_rising;
  attachInterrupt(digitalPinToInterrupt(rear_bottom_dist_pin), ch8_rising_interrupt, RISING);
}

//ch9
void ch9_rising_interrupt(){
  ch_9_rising = micros();
  attachInterrupt(digitalPinToInterrupt(left_dist_pin), ch9_falling_interrupt, FALLING);
}

void ch9_falling_interrupt(){
  ch9_duty_cycle = micros()-ch_9_rising;
  attachInterrupt(digitalPinToInterrupt(left_dist_pin), ch9_rising_interrupt, RISING);
}

//ch10
void ch10_rising_interrupt(){
  ch_10_rising = micros();
  attachInterrupt(digitalPinToInterrupt(right_dist_pin), ch10_falling_interrupt, FALLING);
}

void ch10_falling_interrupt(){
  ch10_duty_cycle = micros()-ch_10_rising;
  attachInterrupt(digitalPinToInterrupt(right_dist_pin), ch10_rising_interrupt, RISING);
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
