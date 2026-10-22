#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include "ZLDriverCANOpen.h"

CanFrame txSDOFrame = { 0 };
CanFrame rxSDOFrame = { 0 };
CanFrame txFrame = { 0 };
CanFrame rxFrame = { 0 };

CANIO::CANIO(int CAN_Speed, int TX_Pin, int RX_Pin, int TX_Queue, int RX_Queue)
{
  _CAN_Speed = CAN_Speed;
  _TX_Pin = TX_Pin;
  _RX_Pin = RX_Pin;
  _TX_Queue = TX_Queue;
  _RX_Queue = RX_Queue;
}

ZLDriverCANOpen::ZLDriverCANOpen(int TX_ID, int RX_ID)
{
  _TX_ID = TX_ID;
  _RX_ID = RX_ID;
}

bool CANIO::begin() {
  return ESP32Can.begin(ESP32Can.convertSpeed(_CAN_Speed), _TX_Pin, _RX_Pin, _TX_Queue, _RX_Queue);
}

void CANIO::readCmdVelFromCAN(bool* gotCmdVelData, bool* openOrCloseLoop, float* xCmdVel, float* yCmdVel, float* wCmdVel){
  bool openOrCloseLoopBuf = false;
  uint8_t xSpdBuf = 0, ySpdBuf = 0, wSpdBuf = 0;
  uint8_t xDirBuf = 0, yDirBuf = 0, wDirBuf = 0;

  if(ESP32Can.readFrame(rxFrame, 0)) {
    if (rxFrame.identifier == 0x701 || rxFrame.identifier == 0x702){
      openOrCloseLoopBuf = (rxFrame.identifier == 0x702);
      xSpdBuf = rxFrame.data[0];
      ySpdBuf = rxFrame.data[1];
      wSpdBuf = rxFrame.data[2];
      xDirBuf = rxFrame.data[3];
      yDirBuf = rxFrame.data[4];
      wDirBuf = rxFrame.data[5];  
      *gotCmdVelData = true;
    }
    else {
      *gotCmdVelData = false;
    }
  } else {
    *gotCmdVelData = false;
  }

  *openOrCloseLoop = openOrCloseLoopBuf;
  *xCmdVel = xSpdBuf * (xDirBuf == 0 ? -1 : 1) / 100.0;
  *yCmdVel = ySpdBuf * (yDirBuf == 0 ? -1 : 1) / 100.0;
  *wCmdVel = wSpdBuf * (wDirBuf == 0 ? -1 : 1) / 100.0;
}

void CANIO::writeCmdVelFromCAN(bool openOrCloseLoop, float xCmdVel, float yCmdVel, float wCmdVel)
{
  int xPosBuf = xCmdVel * 100;
  int yPosBuf = yCmdVel * 100;
  int wPosBuf = wCmdVel * 100;

  txFrame.identifier = !openOrCloseLoop ? 0x701 : 0x702;
  txFrame.extd = 0;
  txFrame.data_length_code = 8;
  txFrame.data[0] = abs(xPosBuf);
  txFrame.data[1] = abs(yPosBuf);
  txFrame.data[2] = abs(wPosBuf);
  txFrame.data[3] = (xPosBuf < 0) ? 0 : 1;    
  txFrame.data[4] = (yPosBuf < 0) ? 0 : 1;    
  txFrame.data[5] = (wPosBuf < 0) ? 0 : 1;    
  txFrame.data[6] = 0;
  txFrame.data[7] = 0;
  ESP32Can.writeFrame(txFrame, 1);
}

void ZLDriverCANOpen::canSendSDO(uint16_t nodeID, int byteLength, uint16_t index, uint8_t subIndex, int32_t value) {
  txSDOFrame.identifier = nodeID;
  txSDOFrame.extd = 0;
  txSDOFrame.rtr = 0;
  txSDOFrame.data_length_code = 8;
  
  switch(byteLength){
    case -1: txSDOFrame.data[0] = 0x40; break; 
    case 1:  txSDOFrame.data[0] = 0x2F; break; 
    case 2:  txSDOFrame.data[0] = 0x2B; break; 
    case 3:  txSDOFrame.data[0] = 0x27; break; 
    case 4:  
    default: txSDOFrame.data[0] = 0x23; break; 
  }

  txSDOFrame.data[1] = (uint8_t)(index & 0xFF);
  txSDOFrame.data[2] = (uint8_t)(index >> 8);
  txSDOFrame.data[3] = subIndex;

  if (byteLength == -1) {
    txSDOFrame.data[4] = 0; txSDOFrame.data[5] = 0;
    txSDOFrame.data[6] = 0; txSDOFrame.data[7] = 0;
  } else {
    txSDOFrame.data[4] = (uint8_t)(value & 0xFF);
    txSDOFrame.data[5] = (uint8_t)((value >> 8) & 0xFF);
    txSDOFrame.data[6] = (uint8_t)((value >> 16) & 0xFF);
    txSDOFrame.data[7] = (uint8_t)((value >> 24) & 0xFF);
  }

  ESP32Can.writeFrame(txSDOFrame, 10);
}

void ZLDriverCANOpen::canReceiveSDO(uint16_t nodeID, uint16_t timeout) {
  bool received = ESP32Can.readFrame(rxSDOFrame, timeout);
  while (received) {
    processReceivedFrame(rxSDOFrame);
    received = ESP32Can.readFrame(rxSDOFrame, 0);
  }
}

void ZLDriverCANOpen::processReceivedFrame(const CanFrame& frame) {
  if (frame.identifier != _RX_ID) return;

  uint16_t checkIndex = (frame.data[2] << 8) | frame.data[1];
  uint8_t checkSubIndex = frame.data[3];

  if (checkIndex == 0x606C) {
    int32_t raw_rpm = (int32_t)((frame.data[7] << 24) | (frame.data[6] << 16) | (frame.data[5] << 8) | frame.data[4]);
    if (checkSubIndex == 1) pLW_RPM = raw_rpm * 0.1f;
    else if (checkSubIndex == 2) pRW_RPM = raw_rpm * 0.1f;
  }
  else if (checkIndex == 0x2032) {
    int16_t raw_temp = (int16_t)((frame.data[5] << 8) | frame.data[4]);
    if (checkSubIndex == 1) pLW_TEMP = raw_temp * 0.1f;
    else if (checkSubIndex == 2) pRW_TEMP = raw_temp * 0.1f;
    else if (checkSubIndex == 3) pD_TEMP = raw_temp * 0.1f;
  }
  else if (checkIndex == 0x6040) {
    if (frame.data[0] == 0x60) _initialized = true;
  }
}

void ZLDriverCANOpen::parseCANOpen() {
  canReceiveSDO(_RX_ID, 0);
}

void ZLDriverCANOpen::readSpeed(float* LW_RPM, float* RW_RPM) {
  canSendSDO(_TX_ID, -1, 0x606C, 0x01, 0x00);
  canSendSDO(_TX_ID, -1, 0x606C, 0x02, 0x00);
  *LW_RPM = pLW_RPM;
  *RW_RPM = pRW_RPM;
}

void ZLDriverCANOpen::writeSpeed(int LW_RPM, int RW_RPM) {
  canSendSDO(_TX_ID, 4, 0x60FF, 0x01, LW_RPM);
  canSendSDO(_TX_ID, 4, 0x60FF, 0x02, RW_RPM);
}

void ZLDriverCANOpen::readTemp(float* LW_TEMP, float* RW_TEMP, float* D_TEMP) {
  canSendSDO(_TX_ID, -1, 0x2032, 0x01, 0x00);
  *LW_TEMP = pLW_TEMP;
  canSendSDO(_TX_ID, -1, 0x2032, 0x02, 0x00);
  *RW_TEMP = pRW_TEMP;
  canSendSDO(_TX_ID, -1, 0x2032, 0x03, 0x00);
  *D_TEMP = pD_TEMP;
}

bool ZLDriverCANOpen::begin() {
  // 1. Fault Reset
  canSendSDO(_TX_ID, 2, 0x6040, 0x00, 0x80); delay(50);
  
  // 2. Configure Modes & Accel
  canSendSDO(_TX_ID, 2, 0x200F, 0x00, 0x00); delay(30); // Asynchronous
  canSendSDO(_TX_ID, 1, 0x6060, 0x00, 0x03); delay(30); // Speed mode
  canSendSDO(_TX_ID, 4, 0x6083, 0x01, 100);  delay(30); // Left Accel
  canSendSDO(_TX_ID, 4, 0x6083, 0x02, 100);  delay(30); // Right Accel
  canSendSDO(_TX_ID, 4, 0x6084, 0x01, 100);  delay(30); // Left Decel
  canSendSDO(_TX_ID, 4, 0x6084, 0x02, 100);  delay(30); // Right Decel

  // 3. Complete CiA 402 Enable Sequence (0x06 -> 0x07 -> 0x0F)
  canSendSDO(_TX_ID, 2, 0x6040, 0x00, 0x06); delay(40);
  canSendSDO(_TX_ID, 2, 0x6040, 0x00, 0x07); delay(40);
  canSendSDO(_TX_ID, 2, 0x6040, 0x00, 0x0F); delay(40);

  return true;
}