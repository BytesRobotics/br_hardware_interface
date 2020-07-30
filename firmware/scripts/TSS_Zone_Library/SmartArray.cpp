#include "SmartArray.h"

unsigned long SmartArray::get_element(unsigned int index) {
  return arr[(index + current_index_) % array_size_];
}

void SmartArray::add_element(unsigned long element) {
  current_index_++;
  current_index_ %= array_size_; //
  current_sum_ -= arr[current_index_];
  current_sum_of_differences_ -= (float(arr[(current_index_ + 1) % array_size_]) - arr[current_index_]);

  arr[current_index_] = element;

  current_sum_ += arr[current_index_];
  current_sum_of_differences_ += (float(arr[current_index_]) - arr[(current_index_ - 1) % array_size_]);
}

unsigned long* SmartArray::get_array() {
  return arr;
}

unsigned long SmartArray::get_average() {
  return current_sum_ / array_size_;
}

unsigned long SmartArray::get_filtered_value(int filter_length) {
  unsigned long sum = 0;
  for (int i = 0; i < 2; i++) { //do this code 'filter length' times
    sum += get_element(-1); //sum = sum + past sensor value(i)
  }
  return sum / 2;
  return get_element(0);
}
