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
    //Max and min here
    double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 0.5, kMinOutput = -0.5;
    //controls error and stuff ykyk
    //no pointer? aight
    m_LeftPIDController.SetP(kP);
    m_LeftPIDController.SetI(kI);
    m_LeftPIDController.SetD(kD);
    //What is IZone and FF?
    m_LeftPIDController.SetIZone(kIz);
    m_LeftPIDController.SetFF(kFF);
    m_LeftPIDController.SetOutputRange(kMinOutput, kMaxOutput);
    
    //Velocity is set at 100RPM
    m_LeftPIDController.SetReference(40, rev::ControlType::kVelocity);
    double velocity = m_LeftEncoder.GetVelocity();

    //Runs a position control loop for 10 loops
    m_LeftPIDController.SetReference(10.0, rev::ControlType::kPosition);

    //4 inch wheel on a 15:1 reduction
    m_LeftEncoder.SetPositionConversionFactor((M_PI * 4) / 15);
    //lets go one foot
    
}

void Robot::AutonomousPeriodic() {
  //maybe we can do tick conversion stuff later
    m_LeftPIDController.SetReference(6.0, rev::ControlType::kPosition);
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
