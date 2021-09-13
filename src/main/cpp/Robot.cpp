// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>


void Robot::RobotInit() {
  m_leftLeadMotor->SetInverted(true);
  m_leftFollowMotor->Follow(*m_leftLeadMotor, false);
  m_rightLeadMotor->SetInverted(false);
  m_rightFollowMotor->Follow(*m_rightLeadMotor, false);

  m_leftLeadMotor->RestoreFactoryDefaults();
  m_rightLeadMotor->RestoreFactoryDefaults();
  m_leftFollowMotor->RestoreFactoryDefaults();
  m_rightFollowMotor->RestoreFactoryDefaults();
}
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("x", stick->GetRawAxis(4));
  frc::SmartDashboard::PutNumber("y ", -stick->GetRawAxis(1));
  frc::SmartDashboard::PutNumber("rotations", Robot::convertDistanceToRots(0.5*(20)*(.03)));
}

void Robot::AutonomousInit() {
  double m_P = 0.5, m_I = 0, m_D = 0.3, kMaxOutput = 0.25, kMinOutput = -0.25;
  
  //Set feet here
  m_leftLeadMotor->GetPIDController().SetP(m_P);
  m_leftLeadMotor->GetPIDController().SetI(m_I);
  m_leftLeadMotor->GetPIDController().SetD(m_D);
  m_leftLeadMotor->GetPIDController().SetOutputRange(kMinOutput, kMaxOutput);

  m_rightLeadMotor->GetPIDController().SetP(m_P);
  m_rightLeadMotor->GetPIDController().SetI(m_I);
  m_rightLeadMotor->GetPIDController().SetD(m_D);
  m_rightLeadMotor->GetPIDController().SetOutputRange(kMinOutput, kMaxOutput);

  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);

  // 15:1 reduction (assumptions), with a 5.7 Diameter wheel
  m_leftEncoder.SetPositionConversionFactor(14/50*(24/40));
  m_rightEncoder.SetPositionConversionFactor(14/50*(24/40));
  prevTime = frc::Timer::GetFPGATimestamp();
  currentPosition = 0;
  currentVelocity = 0;
}
void Robot::AutonomousPeriodic() {
  //Does this work or is there a substantial delay in the init call
  double timeElapsed = frc::Timer::GetFPGATimestamp() - prevTime;
  //adding on velocity that we accelerated because of the timer
  currentVelocity = currentVelocity + (maxAcc*timeElapsed); 
  //currentPosition = ((1/2) * maxAcc * pow(timeElapsed, 2)) + (currentVelocity * timeElapsed);
  
  //Need to derive/check this thing again -> Do later
  //But what this is doing is checking the distance we need to deccelerate (with what's possible)
  distanceToDeccelerate = (std::pow(currentVelocity, 2)/(2*maxAcc));
  
  //If the amount of distance we have is less than distance to deccelerate, reduce velocity, by the most possible
  if(distanceToDeccelerate < positionTotal - currentPosition) {
      //I'm pretty sure once we get to the point, the robot will completely just start going backwards or whatever
      //what do I do to make it stop, or am I already handling that -> I think I am but just make sure
      currentVelocity -= (maxAcc * timeElapsed);
  } 
  else {
    //Else increase velocity
      currentVelocity += (maxAcc * timeElapsed);
      if(currentVelocity > maxVelocity) {
        currentVelocity = maxVelocity;
      }
  //update the current position using velocity
  currentPosition += currentVelocity * timeElapsed;
  }

  //d = vt
  //We want to go in small intervals so we are only going this distance each time until we stop going
  double setPos = currentVelocity * timeElapsed;

  if(currentPosition < positionTotal) {
    m_leftLeadMotor->GetPIDController().SetReference(-Robot::convertDistanceToRots(Robot::convertDistanceToRots(setPos)), rev::ControlType::kPosition);
    m_rightLeadMotor->GetPIDController().SetReference(Robot::convertDistanceToRots(Robot::convertDistanceToRots(setPos)) , rev::ControlType::kPosition);
  }

    //I know that it will always go a little above the feetNeeded, Ill fix it later

      

  prevTime = frc::Timer::GetFPGATimestamp();
  

  //I really dont know what to do with PID and whatnot and how to make it go here    
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  // suggest putting this code into one single method in a new file b/c it's very messy for TeleopPeriodic

  double afterLeftDeadband;
  double afterRightDeadband;
  double leftMotorOutput;
  double rightMotorOutput;

  double joystickY = -stick->GetRawAxis(1); // negate Axis 1, not Axis 4
  double joystickX = stick->GetRawAxis(4);

  if (fabs(joystickX) <= deadband)
    joystickX = 0;
  if (fabs(joystickY) <= deadband)
    joystickY = 0;
  
  double leftAbs = std::fabs(joystickX);
  double rightAbs = std::fabs(joystickY);
  
  // scaling

  //reason for the if statements is that if leftAbs is 0 when its under deadband, the input becomes negative, so it fixes it
  if (leftAbs != 0)
    afterLeftDeadband = (1/(1-deadband)) * leftAbs - (deadband/(1/deadband));
  else
    afterLeftDeadband = 0;

  if (rightAbs != 0)
    afterRightDeadband = (1/(1-deadband)) * rightAbs - (deadband/(1/deadband));
  else
    afterRightDeadband = 0;
  
  joystickX = std::copysign(pow(afterLeftDeadband, 2), joystickX);
  joystickY = std::copysign(pow(afterRightDeadband, 2), joystickY);

  //To fix turning backwards
  if (joystickY >= 0.0) {
    leftMotorOutput = joystickY + joystickX;
    rightMotorOutput = joystickY - joystickX;
  }
  else {
    leftMotorOutput = joystickY - joystickX;
    rightMotorOutput = joystickY + joystickX;
  }
  
  frc::SmartDashboard::PutNumber("leftMotorOutput", leftMotorOutput);
  frc::SmartDashboard::PutNumber("rightMotorOutput", rightMotorOutput);

  m_leftLeadMotor->Set(-leftMotorOutput);
  //negate here
  m_rightLeadMotor->Set(rightMotorOutput);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
