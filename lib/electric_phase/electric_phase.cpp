
#include "electric_phase.h"

ElectricPhase::ElectricPhase(int interrupt_pin){
  this->interrupt_pin = interrupt_pin;
  init();
}
// Initialize the pins - private function
void ElectricPhase::init(){
  pinMode(interrupt_pin, INPUT);
}

// Add impulse when interrupt ocur
void ElectricPhase::addCount(){
  this->count++;
}

// Get total impulses
uint32_t ElectricPhase::getCount(){
  return this->count;
}

// Clear all impulses. The function has to be called after measurement.
void ElectricPhase::clearCount(){
  this->count = 0;
}

// Get consumption in kwh for phase
double ElectricPhase::getConsumptionInctement(){
  if(getCount() == 0){
    return 0;
  } else {
    double increment = getCount() / kwh_impulses;
    addIncrement(increment); // add increment to total consumption
    return increment*100;
  }
}

// Get power of time interval dt = time - last_time
double ElectricPhase::getPower(uint32_t dt){
  if(getCount() == 0){
    return 0;
  } else {
    double value = (getCount()/ w_impulses / dt) * 1000000.0;
    return value;
  }
}
// Get total consumption one decimal place
double ElectricPhase::getTotalConsumption(){
  return (int)(this->total_consumption * 10 + 0.5)/10.0;
}

// Set total consumption after restart
void ElectricPhase::setTotalConsumption(double value){
  this->total_consumption = value;
}

// PRIVATE
void ElectricPhase::addIncrement(double increment){
  this->total_consumption += increment;
}
