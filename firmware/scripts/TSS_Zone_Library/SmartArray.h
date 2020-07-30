#ifndef SMART_ARRAY_H
#define SMART_ARRAY_H

#define SMART_ARRAY_SIZE 2000    //Defines the smart array size. Temporary implementation

//class SmartArray is put int the .cpp because it only needs to be accessed internally by the library
//and not by the end user
class SmartArray { //array class used to produce running average to compare current sensor reading to. At size 2,000, it is averaging over ~1 second
  public:
    unsigned long get_element(unsigned int index);
    void add_element(unsigned long element);
    unsigned long* get_array();
    unsigned long get_average();
    unsigned long get_filtered_value(int filter_length);

  private:
    unsigned long arr[SMART_ARRAY_SIZE] {0};
    const unsigned int array_size_ = SMART_ARRAY_SIZE;
    unsigned int current_index_ = 0;
    unsigned long long current_sum_ = 0;
    float current_sum_of_differences_ = 0;
};

#endif
