
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
  return count;
}

// Clear all impulses. The function has to be called after measurement.
void ElectricPhase::clear(){
  this->count = 0;
}

// Get consumption in kwh for phase
double ElectricPhase::getConsumption(){
  return count / kwh_impulses;
}

// Get power of time interval dt = time - last_time
double ElectricPhase::getPower(uint32_t dt){
  return (count / w_impulses / dt) * 1000000.0;
}