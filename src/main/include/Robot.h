// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include "rev/CANSparkMax.h"
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include <frc/DriverStation.h>

class Robot : public frc::TimedRobot {
 public:
  rev::CANSparkMax * m_leftLeadMotor = new rev::CANSparkMax(3, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax * m_rightLeadMotor = new rev::CANSparkMax(1, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax * m_leftFollowMotor = new rev::CANSparkMax(4, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax * m_rightFollowMotor = new rev::CANSparkMax(2, rev::CANSparkMax::MotorType::kBrushless);

  frc::Joystick *stick = new frc::Joystick(0);

  double deadband = 0.08;

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  double convertDistanceToTicks(double);

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};

double Robot::convertDistanceToTicks(double inches) {
  double diameter = 5.7;
  double ticksPerRevolution = 42;
  double wheelCircumference = 15*M_PI*diameter;
  return (inches/wheelCircumference) * ticksPerRevolution;
}


