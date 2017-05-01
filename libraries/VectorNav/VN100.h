/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef VN_h
#define VN_h

#include <SPI.h>

#define VN_CMD_READ              0x01
#define VN_CMD_WRITE             0x02
#define VN_REG_VPE_MAG_CONFIG    36
#define VN_REG_YPR_IACC_ANGR     240

// #define VN_USE_DMA

enum VN100ReadMode {
  VN_REQUEST_WAIT_READ, VN_REQUEST, VN_READ_REQUEST
};

typedef struct {
  uint8_t dummy[4];
  float yaw, pitch, roll;
  float iaccx, iaccy, iaccz;
  float gyrox, gyroy, gyroz;
} __attribute__ ((packed)) VN100240;

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

    // Test read and request (avoid wait)
    // readReg(VN_REG_YPR_IACC_ANGR, 36, NULL, VN_REQUEST);
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

#if defined(VN_USE_DMA)
      cmd[0] = VN_CMD_READ;
      cmd[1] = reg;
      _SPI.writeDMA(4, cmd);
#else
      // Seems like VN100 likes a small gap between bytes. 
      // Waiting for TXE delays by ~0.3us and that seems ok
      SPI_SendData8(_SPI.SPIx, VN_CMD_READ);
      while(!SPI_I2S_GetFlagStatus(_SPI.SPIx, SPI_I2S_FLAG_TXE));
      SPI_SendData8(_SPI.SPIx, reg);
      while(!SPI_I2S_GetFlagStatus(_SPI.SPIx, SPI_I2S_FLAG_TXE));
      SPI_SendData8(_SPI.SPIx, 0);
      while(!SPI_I2S_GetFlagStatus(_SPI.SPIx, SPI_I2S_FLAG_TXE));
      SPI_SendData8(_SPI.SPIx, 0);
      while(!SPI_I2S_GetFlagStatus(_SPI.SPIx, SPI_I2S_FLAG_TXE));
#endif
      digitalWrite(csPin, HIGH);
      if (mode == VN_REQUEST_WAIT_READ) {
#if defined(VN_USE_DMA)
        // need to wait 50 ms before response (SPI overhead adds some)
        delayMicroseconds(45);
#else
        // need to wait 50 ms before response. Use this delay to clear the RX byte
        uint32_t start = micros();
        while (micros() - start < 50) {
          if (SPI_I2S_GetFlagStatus(_SPI.SPIx, SPI_I2S_FLAG_RXNE))
            SPI_ReceiveData8(_SPI.SPIx);
        }
#endif
      }
      else if (mode == VN_REQUEST)
        return 0;
    }

    if (mode == VN_REQUEST_WAIT_READ || mode == VN_READ_REQUEST) {
      digitalWrite(csPin, LOW);

#if defined(VN_USE_DMA)
      uint8_t tempBuf[N+4];
      _SPI.readDMA(N+4, tempBuf);
      memcpy(resphead, tempBuf, 4);
      memcpy(buf, &tempBuf[4], N);
#else
      uint8_t c;
      for (int i=0; i<N+4; ++i) {
        SPI_SendData8(_SPI.SPIx, 0);
        while(!SPI_I2S_GetFlagStatus(_SPI.SPIx, SPI_I2S_FLAG_RXNE));
        c = SPI_ReceiveData8(_SPI.SPIx);

        if (i<4)
          resphead[i] = c;
        else
          buf[i-4] = c;
      }
#endif
      // delayMicroseconds(1);

      digitalWrite(csPin, HIGH);
      if (mode == VN_READ_REQUEST) {
        // Request again
        delayMicroseconds(1);
#if defined(VN_USE_DMA)
        cmd[0] = VN_CMD_READ;
        cmd[1] = reg;
        _SPI.writeDMA(4, cmd);
        delayMicroseconds(1);//doesn't work if released too soon
#else
        digitalWrite(csPin, LOW);
        SPI_SendData8(_SPI.SPIx, VN_CMD_READ);
        while(!SPI_I2S_GetFlagStatus(_SPI.SPIx, SPI_I2S_FLAG_TXE));
        SPI_SendData8(_SPI.SPIx, reg);
        while(!SPI_I2S_GetFlagStatus(_SPI.SPIx, SPI_I2S_FLAG_TXE));
        SPI_SendData8(_SPI.SPIx, 0);
        while(!SPI_I2S_GetFlagStatus(_SPI.SPIx, SPI_I2S_FLAG_TXE));
        SPI_SendData8(_SPI.SPIx, 0);
        while(!SPI_I2S_GetFlagStatus(_SPI.SPIx, SPI_I2S_FLAG_TXE));
#endif

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
    _SPI.transfer(0x02);
    _SPI.transfer(reg);
    _SPI.transfer(0x00);
    _SPI.transfer(0x00);
    for (int i=0; i<N; ++i)
      _SPI.transfer(args[i]);
    // delayMicroseconds(1);
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
  uint8_t get(float& yaw, float& pitch, float& roll, float& yawd, float& pitchd, float& rolld, float& ax, float& ay, float& az) {
    // VN100: 27 (48bytes) = YPR,MAG,ACC,ANGRATES
    // VN100: 240 (36bytes) = YPR,TRUE_INERTIAL_ACC,ANGRATES

    static float dat[9];
    uint8_t errId = readReg(VN_REG_YPR_IACC_ANGR, 36, (uint8_t *)dat);
    // test read and request
    // uint8_t errId = readReg(VN_REG_YPR_IACC_ANGR, 36, (uint8_t *)dat, VN_READ_REQUEST);
    // problems with reading?
    yaw = radians(dat[0]);
    pitch = radians(dat[1]);
    roll = radians(dat[2]);
    ax = dat[3];
    ay = dat[4];
    az = dat[5];
    yawd = dat[8];
    pitchd = dat[7];
    rolld = dat[6];
    return errId;
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