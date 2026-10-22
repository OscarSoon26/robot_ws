/*
  ZhongLing Technology Hub Motor Driver CAN Open Library Header
  Created by Ooi Soo Jack, 6 July 2025
  Written based on the datasheet of ZLAC8030D motor driver

  ONLY FOR UTAR ROBOCON TEAM USE. DO NOT SHARE UNLESS REQUIRED
*/
#ifndef ZLDriverCANOpen_h
#define ZLDriverCANOpen_h

#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>

class CANIO
{
  public:
    CANIO(int CAN_Speed, int TX_Pin, int RX_Pin, int TX_Queue, int RX_Queue);
    bool begin();
    void readCmdVelFromCAN(bool* gotCmdVelData, bool* openOrCloseLoop, float* xCmdVel, float* yCmdVel, float* wCmdVel);
    void writeCmdVelFromCAN(bool openorCloseLoop, float xCmdVel, float yCmdVel, float wCmdVel);
  private:
    int _CAN_Speed;
    int _TX_Pin;
    int _RX_Pin;
    int _TX_Queue;
    int _RX_Queue;
};

class ZLDriverCANOpen
{
  public:
    ZLDriverCANOpen(int TX_ID, int RX_ID);
    bool begin();
    void canSendSDO(uint16_t nodeID, int byteLength, uint16_t index, uint8_t subIndex, int32_t value);
    void canReceiveSDO(uint16_t nodeID, uint16_t timeout);
    void processReceivedFrame(const CanFrame& frame);
    void parseCANOpen();
    void readSpeed(float* LW_RPM, float* RW_RPM);
    void writeSpeed(int LW_RPM, int RW_RPM);
    void readTemp(float* LW_TEMP, float* RW_TEMP, float* D_TEMP);
    
    float pLW_RPM = 0.0;
    float pRW_RPM = 0.0;
    float pLW_TEMP = 0.0;
    float pRW_TEMP = 0.0;
    float pD_TEMP = 0.0;

  private:
    int _TX_ID;
    int _RX_ID;
    bool _initialized = false;
};

#endif