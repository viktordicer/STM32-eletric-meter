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
  double getConsumptionInctement();
  double getTotalConsumption();
  void setTotalConsumption(double value);


private:
  void init();
  void addIncrement(double increment); // add increment to total Consumption

  int interrupt_pin;
  uint32_t count;
  double kwh_impulses = 2000.0; // Number of impulses per 1kWh
  double w_impulses = 0.555556; // 0,55556 impulses per second => 1 000 W
  double total_consumption; //total consumption in kWh
  double consum_scale = 100.0; // scale for comsumption in kWh. Incrementation of consumption is too small number.
};

#endif //_ELECTRICPHASE_H_