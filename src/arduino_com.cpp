#include "usb_hw_interface/arduino_com.h"

HardwareCom::HardwareCom(const std::string& port, int baud):
        connection_(port, baud, serial::Timeout::simpleTimeout(10),
                    serial::eightbits, serial::parity_even, serial::stopbits_one)
//Serial set to SER_8E1 (8 bit, even parity, 1 stop bit)
{
    //open serial port
    if(connection_.isOpen()){
        std::cout << "Port " << port << " opened successfully!\n";
    } else {
        std::cerr << "Port " << port << " failed to open successfully!\n";
        exit(1);
    }
}

bool HardwareCom::set_controller(double rmotor_cmd, double lmotor_cmd, double head_cmd){
//  std::cout << "rmotor_cmd: " << rmotor_cmd << "\n";
//  std::cout << "lmotor_cmd: " << lmotor_cmd << "\n";
   auto lmotor = static_cast<int>(map<double>(lmotor_cmd, -1, 1, -1000, 1000));
   auto rmotor = static_cast<int>(map<double>(rmotor_cmd, -1, 1, -1000, 1000));
   auto head = static_cast<int>(map<double>(head_cmd, -1, 1, -1000, 1000));

   constrain(head,   -1000, 1000);
   constrain(lmotor, -1000, 1000);
   constrain(rmotor, -1000, 1000);

  //Fill outGoingPacket with four bytes with proper packet structure
   outgoing_packet_[5] = static_cast<uint8_t>(lmotor >> 8);                 //lmotor MSB
   outgoing_packet_[4] = static_cast<uint8_t>(lmotor & 0b0000000011111111); //lmotor LSB
   outgoing_packet_[3] = static_cast<uint8_t>(rmotor >> 8);                 //rmotor  MSB
   outgoing_packet_[2] = static_cast<uint8_t>(rmotor & 0b0000000011111111); //rmotor LSB
   outgoing_packet_[1] = static_cast<uint8_t>(head >> 8);                   //head  MSB
   outgoing_packet_[0] = static_cast<uint8_t>(head & 0b0000000011111111);   //head LSB

   //PEC by XORing all values
   outgoing_packet_[outgoing_packet_length_ - 1] = outgoing_packet_[0];
   for(int i=1; i < outgoing_packet_length_ - 1; i++){ outgoing_packet_[outgoing_packet_length_ - 1]^=outgoing_packet_[i];}

//  std::cout << "packet down: " <<  std::bitset<8>(outgoing_packet_[4]) << std::bitset<8>(outgoing_packet_[3]) << \
//  std::bitset<8>(outgoing_packet_[2]) << std::bitset<8>(outgoing_packet_[1]) << \
//  std::bitset<8>(outgoing_packet_[0]) << "\n";

  size_t bytesSent = connection_.write(outgoing_packet_, outgoing_packet_length_); //Sending 6 bytes
  return int(bytesSent) == outgoing_packet_length_;
}

bool HardwareCom::read_controller(){
    if(static_cast<int>(connection_.available()) > incoming_packet_length_ * 5){
        std::cerr << "Buffer Overflowing" << std::endl;
    }
    if(static_cast<int>(connection_.available()) >= incoming_packet_length_ - 1){
        connection_.read(incoming_packet_, incoming_packet_length_);
        connection_.flushInput();

        //Print out the entire packet
//        std::cout << "Packet received:   ";
//        for(int x=0; x<incoming_packet_length_; x++){
//            std::cout << std::bitset<8>(incoming_packet_[x]);
//        }
//        std::cout << "\n";

        uint8_t pec = incoming_packet_[0];
        for(int i=1; i < incoming_packet_length_-1; i++){ pec^=incoming_packet_[i];}
        if(pec == static_cast<uint8_t>(incoming_packet_[incoming_packet_length_ - 1])){
//            std::cout << "PEC Correct\n";
            /// Fill in channel array with distance sensor values
            for(int i =0; i<2*num_channels_; i+=2){ channels_[i / 2] = static_cast<int>(incoming_packet_[i] | (incoming_packet_[i + 1] << 8));}

            auto o = 2*num_channels_; //! offset
            encoder_left_ = static_cast<int>(incoming_packet_[o] | (incoming_packet_[o+1] << 8) | (incoming_packet_[o+2] << 16) | (incoming_packet_[o+3] << 24));
            encoder_right_ = static_cast<int>(incoming_packet_[o+4] | (incoming_packet_[o+5] << 8) | (incoming_packet_[o+6] << 16) | (incoming_packet_[o+7] << 24));

            latitude_.bytes[0] = incoming_packet_[o+8];
            latitude_.bytes[1] = incoming_packet_[o+9];
            latitude_.bytes[2] = incoming_packet_[o+10];
            latitude_.bytes[3] = incoming_packet_[o+11];

            longitude_.bytes[0] = incoming_packet_[o+12];
            longitude_.bytes[1] = incoming_packet_[o+13];
            longitude_.bytes[2] = incoming_packet_[o+14];
            longitude_.bytes[3] = incoming_packet_[o+15];

            hdop_.bytes[0] = incoming_packet_[o+16];
            hdop_.bytes[1] = incoming_packet_[o+17];
            hdop_.bytes[2] = incoming_packet_[o+18];
            hdop_.bytes[3] = incoming_packet_[o+19];

            speed_    = static_cast<int>(incoming_packet_[o+20] | (incoming_packet_[o+21] << 8)) / 100.0;
            angle_    = static_cast<int>(incoming_packet_[o+22] | (incoming_packet_[o+23] << 8)) / 100.0;
            altitude_ = static_cast<int>(incoming_packet_[o+24] | (incoming_packet_[o+25] << 8)) / 100.0;

            fix_ = static_cast<int>(incoming_packet_[o+26]);
            fix_quality_ = static_cast<int>(incoming_packet_[o+27]);
            satellites_ = static_cast<int>(incoming_packet_[o+28]);

            tss_byte_ = incoming_packet_[o+29];

            return true;
        }
    }
    return false;
}

int HardwareCom::get_dist_sensor_value(DistSensor sensor) {
    int channel = static_cast<int>(sensor);
    int returnVal;
    if(abs(last_channels_[channel] - channels_[channel]) < max_diff_ || channels_last_update_skipped_[channel]){
        // Apply mild filtering
        returnVal = static_cast<int>(round((last_channels_[channel] + 2 * channels_[channel]) / 3.0));
        last_channels_[channel] = returnVal;
        channels_last_update_skipped_[channel] = false;
    } else {
//        std::cout << "Value ignored" << std::endl;
        returnVal = last_channels_[channel];
        // We dont want to skip more that once in a row
        channels_last_update_skipped_[channel] = true;
    }
    return returnVal;
}

long HardwareCom::get_left_encoder() const {
    return encoder_left_;
}

long HardwareCom::get_right_encoder() const {
    return encoder_right_;
}

float HardwareCom::get_latitude() const{
    return latitude_.number;
}

float HardwareCom::get_longitude() const{
    return longitude_.number;
}

float HardwareCom::get_hdop() const{
    return hdop_.number;
}


float HardwareCom::get_speed() const{
    return speed_;
}

float HardwareCom::get_altitude() const{
    return altitude_;
}

float HardwareCom::get_angle() const{
    return angle_;
}

int HardwareCom::get_fix() const{
    return fix_;
}

int HardwareCom::get_fix_quality() const{
    return fix_quality_;
}

int HardwareCom::get_num_satellites() const{
    return satellites_;
}

bool HardwareCom::tss_has_collision(TSS section) const {
    return static_cast<bool>(tss_byte_ & (1 << static_cast<int>(section)));
}


