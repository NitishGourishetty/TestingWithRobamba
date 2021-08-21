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

    m_leftLeadMotor->RestoreFactoryDefaults();
    m_rightLeadMotor->RestoreFactoryDefaults();
    m_leftFollowMotor->RestoreFactoryDefaults();
    m_rightFollowMotor->RestoreFactoryDefaults();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("x", stick->GetX());
  frc::SmartDashboard::PutNumber("y ", stick->GetX());
}

double convertDistanceToTicks (double inches) {
  double radius = 3;
  double ticksPerRevolution = 42;
  double wheelCircumference = 15*2*M_PI*radius;
  return (inches/wheelCircumference) * ticksPerRevolution;
}


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
   m_leftLeadMotor->GetEncoder().SetPosition(0);
   m_rightLeadMotor->GetEncoder().SetPosition(0);

}

void Robot::AutonomousPeriodic() {
  //Encoder PID Stuff 
 
  //What do I put here, im not sure about any of these values
  double m_P = 10;
  double m_I = 10;
  double m_D = 5;
  double ticksPerRevolution = 42;
  double wheelCircumference = 15;
   
   //(inches is distance)
  double ticksNeeded = convertDistanceToTicks(36);
   
   //So 5 feet would be (5/distanceCoveredPerRev ) * tickPerRevolution
  double leftMotorRevs = m_leftLeadMotor->GetEncoder().GetPosition() * ticksPerRevolution;
  double rightMotorRevs = m_rightLeadMotor->GetEncoder().GetPosition() * ticksPerRevolution;

  //which do you check, it shouldn't matter
  if(leftMotorRevs < 100) {
      m_leftLeadMotor->Set(0.5);
      m_rightLeadMotor->Set(0.5);
    }
  }

  //I really dont know what to do with PID and whatnot and how to make it go here


void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  double leftMotorOutput;
  double rightMotorOutput;

  //Are the numbers correct and does it return -128 to 128 kind of cofnused
  // double yStick = stick->GetRawAxis(1);
  // double xStick = stick->GetRawAxis(4);

    //double joystickX = stick->GetX();
    //double joystickY = stick->GetY();
    //Left Y, Right X
    double joystickY = stick->GetRawAxis(1);
    double joystickX = stick->GetRawAxis(4) * -1.0;

    if (fabs(joystickX) <= deadband)
      joystickX = 0;
    if (fabs(joystickY) <= deadband)
      joystickY = 0;

    //copy sign just copies the sign of the 2nd input
    //copysign(-10, 1) will return 10
    //hmm?
  //    double maxSpeed = std::max(std::fabs(joystickX), std::fabs(joystickY));
  //    if(joystickY < 0) 
  //      maxSpeed = -maxSpeed;
  
  // if(joystickY >= 0.0) {
  //     if(joystickX >= 0) {
  //     //1st quadrant
  //     leftMotorOutput=maxSpeed;
  //     rightMotorOutput=joystickY-joystickX;
  //     } else {
  //     //2nd quadrant
  //     leftMotorOutput=joystickY+joystickX;
  //     rightMotorOutput=maxSpeed;
  //     }
  // }
  // else
  // {
  //   if(joystickX >= 0) {
  //   //3rd quadrant
  //   leftMotorOutput=joystickX+joystickY;
  //   rightMotorOutput=maxSpeed;
  //     } else {
  //   //4th quadrant
  //   leftMotorOutput=maxSpeed;
  //   rightMotorOutput=joystickY-joystickX;
  //     }
  // }
  


  //Not working right now-figure out later
  //AFJOBWFIOFIFIFIFIFIFIFBIFIFOBIBFBFBBFB
  // leftMotorOutput = joystickX + joystickY;
  // rightMotorOutput = joystickX - joystickY;

  frc::SmartDashboard::PutNumber("leftMotor", leftMotorOutput);
  frc::SmartDashboard::PutNumber("rightMotor", rightMotorOutput);
  
  double leftAbs = std::abs(joystickX);
  double rightAbs = std::abs(joystickY);
  
  // leftMotorOutput = (1/(1-deadband)) * leftAbs - (deadband/(1/deadband));
  
  // rightMotorOutput = (1/(1-deadband)) * rightAbs - (deadband/(1/deadband));
  // leftMotorOutput = std::copysign(leftMotorOutput*leftMotorOutput ,leftMotorOutput);
  // rightMotorOutput = std::copysign(rightMotorOutput*rightMotorOutput ,rightMotorOutput);
  //So it doesnt lose sign

  double afterleftDeadBand = (1/(1-deadband)) * leftAbs - (deadband/(1/deadband));
  double afterRightDeadBand = (1/(1-deadband)) * rightAbs - (deadband/(1/deadband));
  joystickX = std::copysign(afterleftDeadBand*afterleftDeadBand ,joystickX);
  joystickY = std::copysign(afterRightDeadBand*afterRightDeadBand ,joystickY);
  leftMotorOutput = joystickX + joystickY;
  rightMotorOutput = joystickY - joystickX;
  
  //deadband type stuff yk yk
  
  m_leftLeadMotor->Set(leftMotorOutput);
  m_rightLeadMotor->Set(rightMotorOutput * -1.0);

  // myDrive.ArcadeDrive(-driveStick.GetY(), driveStick.GetX());, could I just do this???????
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
