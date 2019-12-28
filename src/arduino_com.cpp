#include "usb_hw_interface/arduino_com.h"

/**
* @breif constrians value between min and max inclusive. Value is returned by reference.
* @param[in,out] value input to be constrianed
* @param[in] min The minimum value that "value" should be able to be
* @param[in] max The maximum value that "value" should be able to be
*/
template <class T>
void constrain(T &value, T min, T max){
    if(value > max){
        value = max;
    } else if(value < min){
        value = min;
    }
}

/**
* @breif returns a number mapped proportioanlly from one range of numbers to another
* @param[in] input Value to be mapped
* @param[in] inMax The maximum value for the range of the input
* @param[in] inMin The minimum value for the range of the input
* @param[in] outMin The minimum value for the range of the output
* @param[in] outMax The maximum value for the range of the output
* @return The input trnslated proportionally from range in to range out
*/
template <class T>
T map(T input, T inMin, T inMax, T outMin, T outMax){
    T output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    return output;
}


HardwareCom::HardwareCom(std::string port, int baud): connection(port, baud, serial::Timeout::simpleTimeout(10),
                                                                 serial::eightbits, serial::parity_even, serial::stopbits_one)
//Serial set to SER_8E1 (8 bit, even parity, 1 stop bit)
{
  //open serial port
  if(connection.isOpen()){
    std::cout << "Port " << port << " opened successfully!\n";
  } else {
    std::cerr << "Port " << port << " failed to open successfully!\n";
    exit(1);
  }
}

bool HardwareCom::setController(double rmotor_cmd, double lmotor_cmd, double head_cmd){
//  std::cout << "rmotor_cmd: " << rmotor_cmd << "\n";
//  std::cout << "lmotor_cmd: " << lmotor_cmd << "\n";
   auto lmotor = static_cast<int>(map<double>(lmotor_cmd, -1, 1, -1000, 1000));
   auto rmotor = static_cast<int>(map<double>(rmotor_cmd, -1, 1, -1000, 1000));
   auto head = static_cast<int>(map<double>(head_cmd, -1, 1, -1000, 1000));

   constrain(head,   -1000, 1000);
   constrain(lmotor, -1000, 1000);
   constrain(rmotor, -1000, 1000);

  //Fill outGoingPacket with four bytes with proper packet structure
   outgoingPacket[5] = static_cast<uint8_t>(lmotor >> 8);                 //lmotor MSB
   outgoingPacket[4] = static_cast<uint8_t>(lmotor & 0b0000000011111111); //lmotor LSB
   outgoingPacket[3] = static_cast<uint8_t>(rmotor >> 8);                 //rmotor  MSB
   outgoingPacket[2] = static_cast<uint8_t>(rmotor & 0b0000000011111111); //rmotor LSB
   outgoingPacket[1] = static_cast<uint8_t>(head >> 8);                   //head  MSB
   outgoingPacket[0] = static_cast<uint8_t>(head & 0b0000000011111111);   //head LSB

   //PEC by XORing all values
   outgoingPacket[outgoingPacketLength-1] = outgoingPacket[0];
   for(int i=1; i<outgoingPacketLength-1;i++){outgoingPacket[outgoingPacketLength-1]^=outgoingPacket[i];}

//  std::cout << "packet down: " <<  std::bitset<8>(outgoingPacket[4]) << std::bitset<8>(outgoingPacket[3]) << std::bitset<8>(outgoingPacket[2]) << std::bitset<8>(outgoingPacket[1]) << std::bitset<8>(outgoingPacket[0]) << "\n";

  size_t bytesSent = connection.write(outgoingPacket, outgoingPacketLength); //Sending 6 bytes

  if(bytesSent != outgoingPacketLength){
//    std::cerr << "Number of bytes (" << bytesSent << ") sent not equal to six!\n";
    return false;
  }
//  std::cout << "packet sent\n";
  return true;
}

bool HardwareCom::readController(){
    if(connection.available()>=incomingPacketLength){
        connection.read(incomingPacket, incomingPacketLength);
        connection.flushInput();

        //Print out the entire packet
//        std::cout << "Packet received:   ";
//        for(int x=0; x<incomingPacketLength; x++){
//            std::cout << std::bitset<8>(incomingPacket[x]);
//        }
//        std::cout << "\n";

        uint8_t pec = incomingPacket[0];
        for(int i=1; i<incomingPacketLength-1;i++){pec^=incomingPacket[i];}
        if(pec == static_cast<uint8_t>(incomingPacket[incomingPacketLength-1])){
//            std::cout << "PEC Correct\n";
            //Fill in channel array with distance sensor values
            for(int i =0; i<8; i+=2){channels[i/2] = static_cast<int>(incomingPacket[i] | (incomingPacket[i+1]<<8));}

            encoderLeft = static_cast<int>(incomingPacket[10] | (incomingPacket[11]<<8) | (incomingPacket[12]<<16) | (incomingPacket[13]<<24));
            encoderRight = static_cast<int>(incomingPacket[14] | (incomingPacket[15]<<8) | (incomingPacket[16]<<16) | (incomingPacket[17]<<24));

            latitude.bytes[0] = incomingPacket[18];
            latitude.bytes[1] = incomingPacket[19];
            latitude.bytes[2] = incomingPacket[20];
            latitude.bytes[3] = incomingPacket[21];

            longitude.bytes[0] = incomingPacket[22];
            longitude.bytes[1] = incomingPacket[23];
            longitude.bytes[2] = incomingPacket[24];
            longitude.bytes[3] = incomingPacket[25];

            speed = static_cast<int>(incomingPacket[26] | (incomingPacket[27]<<8))/100.0;
            angle = static_cast<int>(incomingPacket[28] | (incomingPacket[29]<<8))/100.0;
            altitude = static_cast<int>(incomingPacket[30] | (incomingPacket[31]<<8))/100.0;

            fix = static_cast<int>(incomingPacket[32]);
            fix_quality = static_cast<int>(incomingPacket[33]);
            satellites = static_cast<int>(incomingPacket[34]);

            return true;
        } //check PEC byte
        return false;
    }
    return false;
}

int HardwareCom::getCh1() {
    return channels[0];
}

int HardwareCom::getCh2() {
    return channels[1];
}

int HardwareCom::getCh3() {
    return channels[2];
}

int HardwareCom::getCh4() {
    return channels[3];
}

int HardwareCom::getCh5() {
    return channels[4];
}

long HardwareCom::getEncoderLeft() {
    return encoderLeft;
}

long HardwareCom::getEncoderRight() {
    return encoderRight;
}

float HardwareCom::getLatitude(){
    return latitude.number;
}

float HardwareCom::getLongitude(){
    return longitude.number;
}

float HardwareCom::getSpeed(){
    return speed;
}

float HardwareCom::getAltitude(){
    return altitude;
}

float HardwareCom::getAngle(){
    return angle;
}

int HardwareCom::getFix(){
    return fix;
}

int HardwareCom::getFixQuality(){
    return fix_quality;
}

int HardwareCom::getNumSatellites(){
    return satellites;
}

