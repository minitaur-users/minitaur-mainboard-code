/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io> 
 * and Turner Topping <turner@ghostrobotics.io>
 */
#ifndef VN_h
#define VN_h

#include <SPI.h>

#define VN_CMD_READ              0x01
#define VN_CMD_WRITE             0x02
#define VN_REG_VPE_MAG_CONFIG    36
#define VN_REG_YPR_IACC_ANGR     240
#define VN_REG_COM_PRTCL_CNTRL   30

enum VN100ReadMode {
  VN_REQUEST_WAIT_READ, VN_REQUEST, VN_READ_REQUEST
};

typedef struct {
  float dat[9];
  uint16_t checksum;
} __attribute__ ((packed)) VN100240CHECKSUM;

// Calculates the 16-bit CRC for the given ASCII or binary message.
unsigned short calculateCRC(unsigned char data[], unsigned int length)
{
  unsigned int i;
  unsigned short crc = 0;
  for(i=0; i<length; i++){
    crc  = (unsigned char)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    crc ^= (unsigned char) (crc & 0xff) >> 4;
    crc ^= (crc << 8) << 4;
    crc ^= ((crc & 0xff) << 4) << 1;
        }
  return crc;
}

unsigned char calculateChecksum (unsigned char data[], unsigned int length)
{
  unsigned int i;
  unsigned char cksum = 0;
  for (i=0;i<length; i++) {
    cksum ^= data[i];
  }
  return cksum;
}

/**
 * @brief VN100 hardware interface library
 */
class VN100 {
  SPIClass& _SPI;
  uint8_t csPin;
  uint8_t cmd[4];
public:
  // response header for SPI read/write commands
  uint8_t resphead[4];
  // VN100240 vn100240pkt, vn100240pkt2;

  /**
   * @brief Construct with reference to which SPI
   * @param _SPI SPI, SPI_2, etc.
   */
  VN100(SPIClass& _SPI) : _SPI(_SPI) {}
  
  /**
   * @brief Send initialization commands to the VN100
   * 
   * @param csPin chip select pin
   */
  void init(uint8_t csPin) {
    this->csPin = csPin;

    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);

    _SPI.begin();
    // APB1 on F303 has prescaler 2 => 36MHz => div2 = 18MHz
    // VN100 says 16MHz max SPI speed
    _SPI.setClockDivider(SPI_CLOCK_DIV2);
    _SPI.setBitOrder(MSBFIRST);
    _SPI.setDataMode(SPI_MODE3);

    // Use DMA
    _SPI.initDMA(RCC_AHBPeriph_DMA1, DMA1_Channel5, DMA1_Channel4, DMA1_FLAG_TC5, DMA1_FLAG_TC4);
    cmd[2] = cmd[3] = 0;//other two are set each time

    // Initial configuration
    // VPE config (register 35) defaults are 1,1,1,1 - OK
    // VPE mag config (36) set all to 0 (don't trust magnetometer)
    float magConfig[9] = {0,0,0, 0,0,0, 0,0,0};
    writeReg(VN_REG_VPE_MAG_CONFIG, 36, (const uint8_t *)magConfig);
    delay(1000);
    uint8_t crcConfig[7] = {0,0,0,0,1,3,0};
    writeReg(VN_REG_COM_PRTCL_CNTRL,7,crcConfig);
    //Write 16 bit CRC
    //B0 = 0 (Serial COunt OFF), B1 = 0 (Serial Status OFF), B2 = 0 (SPICount OFf)
    //B3 = 0 (SPIStatus OFF), B4 = 1 (Serial Checksum), B5 = 3 (SPI 16bit Checksum )
    //B6 = 0 (Send Error)
    // uint8_t crcConfig[7] = {0,0,0,0,1,3,0};
    // writeReg(VN_REG_COM_PRTCL_CNTRL,7,crcConfig);

    // Test read and request (avoid wait)
    //readReg(VN_REG_YPR_IACC_ANGR, 36, NULL, VN_REQUEST);
  }

  /**
   * @brief Read a register
   * 
   * @param reg Register ID
   * @param N Size of payload packet (in bytes)
   * @param buf Pre-allocated buffer to place payload packet in
   * @param mode VN_REQUEST_WAIT_READ, VN_REQUEST, or VN_READ_REQUEST
   * @return Error ID
   */
  uint8_t readReg(uint8_t reg, int N, uint8_t *buf, VN100ReadMode mode=VN_REQUEST_WAIT_READ) {
    if (mode == VN_REQUEST_WAIT_READ || mode == VN_REQUEST) {
      // Request
      digitalWrite(csPin, LOW);

      cmd[0] = VN_CMD_READ;
      cmd[1] = reg;
      // _SPI.writeDMA(4, cmd); NON-crc
      uint16_t crc = calculateCRC(cmd,4);//Calculate checksum of read command
      uint8_t *pCRC = (uint8_t*)&crc;//set up dummy for endian swap
      swapByte(&pCRC[0], &pCRC[1]); //Swap checksum bytes
      uint8_t cmdpcrc[6]; //create cmd plus crc
      memcpy(&cmdpcrc[0], &cmd, 4);//populate with cmd
      memcpy(&cmdpcrc[4], &crc, 2);//Append checksum
      _SPI.writeDMA(6,cmdpcrc); //write it

      digitalWrite(csPin, HIGH);
      if (mode == VN_REQUEST_WAIT_READ) {
        // need to wait 45 ms before response (SPI overhead adds some)
        delayMicroseconds(45);
      }
      else if (mode == VN_REQUEST)
        return 0;
    }

    if (mode == VN_REQUEST_WAIT_READ || mode == VN_READ_REQUEST) {
      digitalWrite(csPin, LOW);

      /* // NON-CRC 
      // uint8_t tempBuf[N+4];
      // _SPI.readDMA(N+4, tempBuf);
      // memcpy(resphead, tempBuf, 4);
      // memcpy(buf, &tempBuf[4], N);
      */
      uint8_t tempBuf[N+6];
      _SPI.readDMA(N+6,tempBuf);
      memcpy(resphead,tempBuf,4);
      memcpy(buf, &tempBuf[4],N+2);
          
      // delayMicroseconds(1);

      digitalWrite(csPin, HIGH);
      if (mode == VN_READ_REQUEST) {
        // Request again
        delayMicroseconds(30);
        digitalWrite(csPin, LOW);
        cmd[0] = VN_CMD_READ;
        cmd[1] = reg;
        // _SPI.writeDMA(4, cmd); NON-crc version
        uint16_t crc = calculateCRC(cmd,4);//Calculate checksum of read command
        uint8_t *pCRC = (uint8_t*)&crc;//set up dummy for endian swap
        swapByte(&pCRC[0], &pCRC[1]); //Swap checksum bytes
        uint8_t cmdpcrc[6]; //create cmd plus crc
        memcpy(&cmdpcrc[0], &cmd, 4);//populate with cmd
        memcpy(&cmdpcrc[4], &crc, 2);//Append checksum
        _SPI.writeDMA(6,cmdpcrc); //write it
        delayMicroseconds(1);//doesn't work if released too soon

        digitalWrite(csPin, HIGH);
      }
      return resphead[3];
    }
    // should never get here since mode is 0, 1, or 2
    return 0;
  }
  /**
   * @brief Write a register
   * 
   * @param reg Register ID
   * @param N Size of payload packet (in bytes)
   * @param args Payload packet
   * @return Error code in response packet (0 is good)
   */
  uint8_t writeReg(uint8_t reg, int N, const uint8_t *args) {
    digitalWrite(csPin, LOW);
    // delayMicroseconds(1);
    uint8_t tempBuf[N+4];
    tempBuf[0] = 0x02;
    tempBuf[1] = reg;
    tempBuf[2] = 0x00;
    tempBuf[3] = 0x00;
    memcpy(&tempBuf[4], args, N);
    _SPI.writeDMA(N+4, tempBuf);
    // _SPI.transfer(0x02);
    // _SPI.transfer(reg);
    // _SPI.transfer(0x00);
    // _SPI.transfer(0x00);
    // for (int i=0; i<N; ++i)
    //   _SPI.transfer(args[i]);
    // // delayMicroseconds(1);
    digitalWrite(csPin, HIGH);
    delayMicroseconds(50);

    digitalWrite(csPin, LOW);
    // delayMicroseconds(1);
    // "it is sufficient to just clock in only four bytes
    // on the response packet to verify that the write register took effect, 
    // which is indicated by a zero error code."
    for (int i=0; i<4; ++i) {
      resphead[i] = _SPI.transfer(0x00);
    }
    // delayMicroseconds(1);
    digitalWrite(csPin, HIGH);
    return resphead[3];
  }

  uint8_t writeRegCrc(uint8_t reg, int N, const uint8_t *args) {
    digitalWrite(csPin, LOW);
    // delayMicroseconds(1);
    uint8_t tempBuf[N+6];
    tempBuf[0] = 0x02;
    tempBuf[1] = reg;
    tempBuf[2] = 0x00;
    tempBuf[3] = 0x00;
    memcpy(&tempBuf[4], args, N);
    uint16_t crc = calculateCRC(tempBuf,N+4); //Calc crc
    uint8_t *pCRC = (uint8_t*)&crc; //create dummy for swap
    swapByte(&pCRC[0],&pCRC[1]); //Swap for endianness
    memcpy(&tempBuf[N+4],&crc,2); // append crc
    _SPI.writeDMA(N+6, tempBuf); //write
    // _SPI.transfer(0x02);
    // _SPI.transfer(reg);
    // _SPI.transfer(0x00);
    // _SPI.transfer(0x00);
    // for (int i=0; i<N; ++i)
    //   _SPI.transfer(args[i]);
    // // delayMicroseconds(1);
    digitalWrite(csPin, HIGH);
    delayMicroseconds(50);

    digitalWrite(csPin, LOW);
    // delayMicroseconds(1);
    // "it is sufficient to just clock in only four bytes
    // on the response packet to verify that the write register took effect, 
    // which is indicated by a zero error code."
    for (int i=0; i<4; ++i) {
      resphead[i] = _SPI.transfer(0x00);
    }
    // delayMicroseconds(1);
    digitalWrite(csPin, HIGH);
    return resphead[3];
  }

  /**
   * @brief Retrieves angles and angular rates
   * @details This function blocks for a few hundred microseconds.
   * 
   * @param yaw in radians
   * @param pitch in radians
   * @param roll in radians
   * @param yawd in rad/s
   * @param pitchd in rad/s
   * @param rolld in rad/s
   * @param ax true inertical acc in m/s^2
   * @param ay true inertical acc in m/s^2
   * @param az true inertical acc in m/s^2
   */
  bool get(float& yaw, float& pitch, float& roll, float& yawd, float& pitchd, float& rolld, float& ax, float& ay, float& az) {
    // VN100: 27 (48bytes) = YPR,MAG,ACC,ANGRATES
    // VN100: 240 (36bytes) = YPR,TRUE_INERTIAL_ACC,ANGRATES

    // static float dat[9];
    VN100240CHECKSUM packet;
    readReg(VN_REG_YPR_IACC_ANGR, 36, (uint8_t *)&packet);
    // test read and request
    // uint8_t errId = readReg(VN_REG_YPR_IACC_ANGR, 36, (uint8_t *)dat, VN_READ_REQUEST);
    // problems with reading?
    yaw = radians(packet.dat[0]);
    pitch = radians(packet.dat[1]);
    roll = radians(packet.dat[2]);
    ax = packet.dat[3];
    ay = packet.dat[4];
    az = packet.dat[5];
    yawd = packet.dat[8];
    pitchd = packet.dat[7];
    rolld = packet.dat[6];
    uint8_t packetDat[40];
    memcpy(&packetDat[4],&packet,36);
    uint8_t refHeader[4] = {0,1,VN_REG_YPR_IACC_ANGR,0};
    memcpy(&packetDat[0],&refHeader,4);
    uint16_t crc = calculateCRC(packetDat, 40);
    uint8_t *pCRC = (uint8_t*)&crc; //create dummy for swap
    swapByte(&pCRC[0],&pCRC[1]); //Swap for endianness
    // crcMat
    return crc == packet.checksum;
  }

  /**
   * @brief Retrieves angles and angular rates
   * @details This function blocks for a few hundred microseconds.
   * 
   * @param yaw in radians
   * @param pitch in radians
   * @param roll in radians
   * @param yawd in rad/s
   * @param pitchd in rad/s
   * @param rolld in rad/s
   */
  uint8_t get(float& yaw, float& pitch, float& roll, float& yawd, float& pitchd, float& rolld) {
    float ax, ay, az;// dummies
    return get(yaw, pitch, roll, yawd, pitchd, rolld, ax, ay, az);
  }
};


#endif