/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "logic-steering.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <cstring>
#include <vector>
#include <string>
#include <ctime>

//#include <stdlib.h>

float Steering::decode(const std::string &data) noexcept {
    std::cout << "[UDP] Got data:" << data << std::endl;
    float temp = std::stof(data);
    return temp;
}

Steering::Steering(bool verbose, uint32_t id, float pconst, float iconst, float tolerance, cluon::OD4Session &od4, cluon::OD4Session &od4Gpio, cluon::OD4Session &od4Analog, cluon::OD4Session &od4Pwm, std::map<std::string, std::string> commandlineArguments)
    : m_od4(od4)
    , m_od4Gpio(od4Gpio)
    , m_od4Analog(od4Analog)
    , m_od4Pwm(od4Pwm)
    , m_debug(verbose)
    , m_bbbId(id)
    , m_senderStampOffsetGpio(id*1000)
    , m_senderStampOffsetAnalog(id*1000+200)
    , m_senderStampOffsetPwm(id*1000+300)
    , m_initialised()
    , m_groundSteeringRequest()
    , m_steeringCurrentDuty()
    //, m_steerLeft(1)
    , m_steerRight(1)
    //, m_steerSelect(0)
    , m_steerCurrent()
    , m_steerPosition()
    , m_steerPositionRack()
    , m_steerVoltage()
    , m_iControlOld()
    , m_pConst(pconst)
    , m_iConstTI(iconst)
    , m_tolerance(tolerance)
    , m_clamped()
    , m_steeringCurrentDutyOld()
    , m_clampedOld()
    , m_steerRightOld()
    , m_rackFound()
    , m_findRackSeqNo()
    , m_findRackTuning()
    , m_asms()
    , m_clampExtended()
    , m_currentState()
    , m_pressureServiceTank()
    , steerError(0.0)
    
    // Low pass filter for potentionmeter readings
    , beta(0.025f)

{
	Steering::setUp();
	deadzoneError = static_cast<float>(std::stof(commandlineArguments["deadzoneError"]));
	deadzoneDuty = static_cast<float>(std::stof(commandlineArguments["deadzoneDuty"]));
	acceptableError = static_cast<float>(std::stof(commandlineArguments["acceptableError"]));
}

Steering::~Steering() 
{
  Steering::tearDown();
}

void Steering::body()
{
    m_steeringCurrentDuty = 0;
    //for testing the new stm solution
    m_currentState = asState::AS_DRIVING;
    
    if (m_rackFound && (m_currentState == asState::AS_DRIVING)){
    
        controlPosition(m_groundSteeringRequest, m_steerPositionRack);
    }
    else if (m_asms){ //removed pressure check for testing
        findRack();
    }

    if (!m_asms || m_currentState == asState::AS_FINISHED){
        m_findRackSeqNo = 0;
        m_rackFound = false;
        m_clamped = false;
        m_findRackTuning = 0;
        controlPosition(m_steerPosition, m_steerPosition);
    }


    cluon::data::TimeStamp sampleTime = cluon::time::now();
    int16_t senderStamp = 0;

    opendlv::proxy::SwitchStateRequest msgGpio;

	  // GPIO Msg
     if(m_steerRight != m_steerRightOld){
        m_steerRightOld = m_steerRight;
    }
    
    // Only send clamp request until clamp is extended
    if(m_findRackSeqNo == 10){
        senderStamp = m_gpioPinClamp + m_senderStampOffsetGpio;
        msgGpio.state(m_clamped);
        m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
        m_clampedOld = m_clamped;
    }

   	// PWM Msg
   if(m_steeringCurrentDuty > 2000){ 
        opendlv::proxy::PulseWidthModulationRequest msgPwm;
        senderStamp = m_pwmPinSteer + m_senderStampOffsetPwm;
        if(m_steerRight) //if steer right, then add big offset. this would be converted back to a negative pwm signal
        // had to do this since standard pwm request message is non-negative
          msgPwm.dutyCycleNs(m_steeringCurrentDuty + 100000);
        else
          msgPwm.dutyCycleNs(m_steeringCurrentDuty);
          
        m_od4Pwm.send(msgPwm, sampleTime, senderStamp);
        m_steeringCurrentDutyOld = m_steeringCurrentDuty;
    }
}

bool Steering::controlPosition(float setPoint, float refPoint)
{
    float maxRackRight = -22.0;
    float maxRackLeft = 22.0;
    
    float maxSteerRight = -25.0;
    float maxSteerLeft = 25.0;
    
    float tolerance;
    if(m_clampExtended)
      tolerance = 0.2;
    else
      tolerance = 0.1;
    bool ret = false;
    //if steering rack is out of range when asms is turned on, return error and do nothing
    if ((setPoint < maxRackRight || setPoint > maxRackLeft)){
      m_steeringCurrentDuty = 0;
    }
    
    //if actuator is out of range, return error and do nothing
    if ((refPoint < maxRackRight || refPoint > maxSteerLeft)){
      m_steeringCurrentDuty = 0;
    }
    //if actuator is already to the left or right most, then dont move to left or right anymore
    if ((refPoint < maxRackRight && setPoint < maxRackRight) || (refPoint > maxRackLeft && setPoint > maxRackLeft)){
      m_steeringCurrentDuty = 0;
    }
     
    if (setPoint < maxRackRight){
        setPoint = maxRackRight;
    }else if (setPoint > maxRackLeft){
        setPoint = maxRackLeft;
    }

    steerError = setPoint-refPoint;
   
    float pControl = m_pConst * steerError;
    if(m_findRackSeqNo == 10)
      pControl = (m_pConst*4) * steerError;
    float iControl = 0;


    m_iControlOld = m_iControlOld + m_iConstTI*steerError;
    if(m_iControlOld > 30000)
      m_iControlOld = 30000;
    else if(m_iControlOld < -30000)
      m_iControlOld = -30000;
    
     // if error is within a tolerance then only use integration, otherwise noise would cause overshoot
    if(abs(steerError) < tolerance){
      ret = true;
      m_iControlOld = 0;
    }
    
    float controlSignal = pControl + m_iControlOld;
    //float controlSignal = 0.0;
    
    m_steeringCurrentDuty = (uint32_t) abs((int)round(controlSignal));

    if (m_steeringCurrentDuty > 50000){
        m_steeringCurrentDuty = 50000;
    }
    
    if(abs(steerError) > tolerance) //only set new direction when error is large enough
      m_steerRight = controlSignal < 0;    
    
    if((abs(steerError) < tolerance && m_rackFound ) )
      m_steeringCurrentDuty = 0;

    if(abs(steerError) < deadzoneError &&  abs(steerError) > acceptableError && m_steeringCurrentDuty < 10000)
    {
      m_steeringCurrentDuty = deadzoneDuty;
      if(m_debug)
        std::cout << "Deadzone steerError = " << m_steeringCurrentDuty << std::endl;
    }

    if (m_debug){
        std::cout << "[LOGIC-STEERING-FINDRACK] Error: " << steerError 
                    << "\t Duty: " << m_steeringCurrentDuty 
                    
                    << "\t Direction: " << m_steerRight
                    << "\t Request: " << setPoint
                    << "\t Refpoint" << refPoint
                    << "\t Measure: " << m_steerPosition 
                    << "\t deadzoneError: " << deadzoneError 
                    << "\t acceptableError: " << acceptableError 
                    << "\t deadzoneDuty: " << deadzoneDuty 
                    << std::endl;
    }
    return ret;

}

void Steering::findRack()
{
/* varialbes meanings:
m_clamped : gpio out to set clamp on/off
m_clampExtended: gpio in from proximity sensor
*/
    switch(m_findRackSeqNo){
        case 0: // 
            m_clamped = false;
            m_rackFound = false;
            if (controlPosition((m_steerPositionRack+(float) 0), m_steerPosition))
            {   
                m_findRackSeqNo = 10;
                findRack_startTime = time (NULL);
            }
            break;

        case 10:
            findRack_currentTime = time (NULL);
            if(m_debug)
              std::cout << "time since end of m_findRackSeqNo = 10: " << findRack_currentTime - findRack_startTime << std::endl;
            if(controlPosition((m_steerPositionRack+(float) 0), m_steerPosition) && findRack_currentTime - findRack_startTime > 5){
            m_clamped = true;
            if(m_debug)
              std::cout << "Clamp set\n";
            }
            
            if (m_clampExtended){
                m_groundSteeringRequest = m_steerPositionRack;
                m_findRackSeqNo = 20;
            }
            break;
        case 20:
	        m_clamped = true;
            m_findRackTuning = 0;
            m_rackFound = true;
            break;
        default:
        break;
    }
}

void Steering::setUp()
{
  m_initialised = true;
}

void Steering::tearDown()
{
}

uint16_t Steering::getGpioPinClampSensor(){
  return m_gpioPinClampSensor;
}

uint16_t Steering::getGpioPinAsms(){
  return m_gpioPinAsms;
}

uint16_t Steering::getAnalogPinSteerPosition(){
  return m_analogPinSteerPosition;
}

uint16_t Steering::getAnalogPinSteerPositionRack(){
  return m_analogPinSteerPositionRack;
}

uint32_t Steering::getSenderStampOffsetGpio(){
  return m_senderStampOffsetGpio;
}

uint32_t Steering::getSenderStampOffsetAnalog(){
  return m_senderStampOffsetAnalog;
}
uint16_t Steering::getAnalogPinServiceTank(){
  return m_analogPinServiceTank;
}

void Steering::setSteerPositionRack(float pos){
    //m_steerPositionRack = m_steerPositionRack - (beta * (m_steerPositionRack - pos));
    m_steerPositionRack = pos;
}
void Steering::setSteerPosition(float pos){
    //m_steerPosition = m_steerPosition - (beta * (m_steerPosition - pos));
    m_steerPosition = pos;
}
void Steering::setGroundSteeringRequest(float pos){
    m_groundSteeringRequest = pos;
}

void Steering::setClampExtended(bool state){
    m_clampExtended = state;
}
void Steering::setAsms(bool state){
    m_asms = state;
}
void Steering::setCurrentState(uint16_t state){
    m_currentState = (asState) state;
}
void Steering::setPressureServiceTank(float pos){
    m_pressureServiceTank = pos;
}

bool Steering::getInitialised(){
  return m_initialised;
}
