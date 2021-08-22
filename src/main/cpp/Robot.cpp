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

//Rev robotics spark max examples
void Robot::AutonomousInit() {
    m_rightLeadMotor->Follow(*m_rightLeadMotor, false);
    m_leftLeadMotor->Follow(*m_leftLeadMotor, false);
    //Max and min here
    double m_P = 0.1, m_I = 1e-4, m_D = 1, kMaxOutput = 0.5, kMinOutput = -0.5;
    //controls error and stuff ykyk
    //didn't dereference, just global variables
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

    m_leftLeadMotor->SetInverted(true);



    //Velocity is set at 40RPM
    m_leftLeadMotor->GetPIDController().SetReference(40, rev::ControlType::kVelocity);

    //4 inch wheel on a 15:1 reduction (assumptionas)
    m_leftLeadMotor->GetEncoder().SetPositionConversionFactor((M_PI * 4) / 15);
    m_rightLeadMotor->GetEncoder().SetPositionConversionFactor((M_PI * 4) / 15);
    
}

void Robot::AutonomousPeriodic() {
  //maybe we can do tick conversion stuff later, if this aint accepted
  //1 foot
  //does this take in tickconversionstuff automatically, or do I say 42
  //with this: //rev::CANEncoder m_LeftEncoder = m_leftLeadMotor->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);
  int x = 1;
  if(x==1) {
    m_leftLeadMotor->GetPIDController().SetReference(6.0 , rev::ControlType::kPosition);
    m_rightLeadMotor->GetPIDController().SetReference(6.0 , rev::ControlType::kPosition);
  }    
  }

  //I really dont know what to do with PID and whatnot and how to make it go here


void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  double leftMotorOutput;
  double rightMotorOutput;
    //Left Y, Right X
    double joystickY = stick->GetRawAxis(1);
    double joystickX = stick->GetRawAxis(4) * -1.0;

    if (fabs(joystickX) <= deadband)
      joystickX = 0;
    if (fabs(joystickY) <= deadband)
      joystickY = 0;

  frc::SmartDashboard::PutNumber("leftMotor", leftMotorOutput);
  frc::SmartDashboard::PutNumber("rightMotor", rightMotorOutput);
  
  double leftAbs = std::fabs(joystickX);
  double rightAbs = std::fabs(joystickY);
  

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
