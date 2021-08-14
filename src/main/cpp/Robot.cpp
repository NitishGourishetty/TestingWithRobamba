// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include <math.h>
void Robot::RobotInit() {
    //I kind of forgot why we invert here, i'm just doing it in case
    m_leftLeadMotor->SetInverted(true);
    m_leftFollowMotor->Follow(*m_leftLeadMotor, false);
    m_rightLeadMotor->SetInverted(false);
    m_rightFollowMotor->Follow(*m_rightLeadMotor, false);


}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  double leftMotorOutput;
  double rightMotorOutput;

  //Are the numbers correct and does it return -128 to 128 kind of cofnused
  // double yStick = stick->GetRawAxis(1);
  // double xStick = stick->GetRawAxis(4);

    double joystickX = stick->GetX();
    double joystickY = stick->GetY();

    if (fabs(joystickX) <= deadband)
      joystickX = 0;
    if (fabs(joystickY) <= deadband)
      joystickY = 0;

    //copy sign just copies the sign of the 2nd input
    //copysign(-10, 1) will return 10
    double maxSpeed = std::max(std::fabs(joystickX), std::fabs(joystickY));
    if(joystickY < 0) 
      maxSpeed = -maxSpeed;
    
  if(joystickY >= 0.0) {
      if(joystickX >= 0) {
      //1st quadrant
      leftMotorOutput=maxSpeed;
      rightMotorOutput=joystickY-joystickX;
      } else {
      //2nd quadrant
      leftMotorOutput=joystickY+joystickX;
      rightMotorOutput=maxSpeed;
      }
  }
  else
  {
    if(joystickX >= 0) {
    //3rd quadrant
    leftMotorOutput=joystickX+joystickY;
    rightMotorOutput=maxSpeed;
      } else {
    //4th quadrant
    leftMotorOutput=maxSpeed;
    rightMotorOutput=joystickY=joystickX;
      }
  }
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
