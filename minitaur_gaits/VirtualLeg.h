/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef VirtualLeg_h
#define VirtualLeg_h

#include <AbstractMotor.h>
#include "HAL.h"

enum VirtualLegMode {
  STAND=0, STANCE=1, FLIGHT=2, //used by all
  FLIGHT_RETRACT=3 //used by trot
};

enum SignalState {
  SIGNAL_NONE, SIGNAL_QUEUE, SIGNAL_LEAP_STANCE, SIGNAL_LEAP_LAND
};

template< int N > class VirtualLeg {
public:
  uint32_t tTD = 0, tLO = 0;
  VirtualLegMode mode = STAND;
  int legi[N];//indices of physical legs
  float speedAccum = 0;
  float speed = 0;

  // init
  virtual void begin(VirtualLegMode initMode) {
    mode = initMode;
    tTD = millis();
    X.xd = 0;
  }
  virtual void begin() {begin(STANCE);}
  virtual void end() {
    mode = STAND;
  }

  // get functions that return average
  inline float getPosition(int i) {
    float thesum=0;
    for (int j=0; j<N; ++j) {
      thesum += leg[legi[j]].getPosition(i);
    }
    return thesum/((float)N);
  }
  inline float getVelocity(int i) {
    float thesum=0;
    for (int j=0; j<N; ++j) {
      thesum += leg[legi[j]].getVelocity(i);
    }
    return thesum/((float)N);
  }
  inline float getSpeed(float pitch) {
    float thesum=0;
    for (int j=0; j<N; ++j) {
      thesum += leg[legi[j]].getSpeed(pitch);
    }
    return thesum/((float)N);
  }
  inline void setGain(AbstractCoord i, float Kp, float Kd) {
    for (int j=0; j<N; ++j)
      leg[legi[j]].setGain(i, Kp, Kd);
  }
  inline void setGain(AbstractCoord i, float Kp) {
    setGain(i, Kp, 0);
  }
  inline float getToeForceRadial() {
    float uth, ur, urtot = 0;
    for (int j=0; j<N; ++j) {
      leg[legi[j]].getToeForce(ur, uth);
      // Something wrong in signs
      if (legi[j]==2 || legi[j]==3)
        ur = -ur;
      urtot += ur;
    }
    return urtot;
  }
};


class LegPair : public VirtualLeg< 2 > {
public:
  LegPair(int i1, int i2) {
    legi[0] = i1;
    legi[1] = i2;
  }

  // MEAN/DIFF FUNCTIONS
  inline void setOpenLoop(AbstractCoord i, float mean, float diff) {
    leg[legi[0]].setOpenLoop(i, mean-diff);
    leg[legi[1]].setOpenLoop(i, mean+diff);
  }
  inline void setPosition(AbstractCoord i, float mean, float diff) {
    leg[legi[0]].setPosition(i, mean-diff);
    leg[legi[1]].setPosition(i, mean+diff);
  }
};

#endif