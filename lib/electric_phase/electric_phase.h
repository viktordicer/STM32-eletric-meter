#ifndef _ELECTRICPHASE_H_
#define _ELECTRICPHASE_H_

#include <Arduino.h>

class ElectricPhase{
public:
  ElectricPhase(int interrupt_pin);
  void addCount();
  uint32_t getCount();
  void clearCount();
  double getPower(uint32_t dt);
  double getConsumption();


private:
  void init();

  int interrupt_pin;
  uint32_t count;
  double kwh_impulses = 2000.0; // Number of impulses per 1kWh
  double w_impulses = 0.55556; // 0,5556 impulses per second => 1 000 W
};

#endif //_ELECTRICPHASE_H_