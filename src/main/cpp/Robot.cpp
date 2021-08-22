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
}

void Robot::AutonomousInit() {
  double m_P = 0.1, m_I = 1e-4, m_D = 1, kMaxOutput = 0.5, kMinOutput = -0.5;

  m_leftLeadMotor->GetPIDController().SetP(m_P);
  m_leftLeadMotor->GetPIDController().SetI(m_I);
  m_leftLeadMotor->GetPIDController().SetD(m_D);
  m_leftLeadMotor->GetPIDController().SetOutputRange(kMinOutput, kMaxOutput);

  m_rightLeadMotor->GetPIDController().SetP(m_P);
  m_rightLeadMotor->GetPIDController().SetI(m_I);
  m_rightLeadMotor->GetPIDController().SetD(m_D);
  m_rightLeadMotor->GetPIDController().SetOutputRange(kMinOutput, kMaxOutput);

  m_rightLeadMotor->GetEncoder().SetPosition(0);
  m_leftLeadMotor->GetEncoder().SetPosition(0);

  // 15:1 reduction (assumptionas)
  m_leftLeadMotor->GetEncoder().SetPositionConversionFactor((M_PI * 5.7) / 15);
  m_rightLeadMotor->GetEncoder().SetPositionConversionFactor((M_PI * 5.7) / 15);
}
void Robot::AutonomousPeriodic() {
  //maybe we can do tick conversion stuff later, if this aint accepted
  //1 foot
  int x = 1;
  if(x==1) {
    m_leftLeadMotor->GetPIDController().SetReference(6.0 , rev::ControlType::kPosition);
    m_rightLeadMotor->GetPIDController().SetReference(6.0 , rev::ControlType::kPosition);
  }
  //I really dont know what to do with PID and whatnot and how to make it go here    
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  // suggest putting this code into one single method in a new file b/c it's very messy for TeleopPeriodic

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
  double afterleftDeadBand = (1/(1-deadband)) * leftAbs - (deadband/(1/deadband));
  double afterRightDeadBand = (1/(1-deadband)) * rightAbs - (deadband/(1/deadband));
  joystickX = std::copysign(pow(afterleftDeadBand, 2), joystickX);
  joystickY = std::copysign(pow(afterRightDeadBand, 2), joystickY);

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

  m_leftLeadMotor->Set(leftMotorOutput);
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
