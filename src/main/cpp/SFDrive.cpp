// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SFDrive.h"
#include <math.h>
#include <frc/Timer.h>
//Maybe smart dashboard if I want, could gobal variable, h file thing too if I want (figure out)

//Excess handler later?
//Review class construction*
SFDrive::SFDrive(rev::CANSparkMax *leftLeadMotor, rev::CANSparkMax *rightLeadMotor,
                 rev::CANSparkMax *leftFollowMotor, rev::CANSparkMax *rightFollowMotor) : leftLeadMotor{leftLeadMotor}, rightLeadMotor{rightLeadMotor},
                                                                                          leftFollowMotor{leftFollowMotor}, rightFollowMotor{rightFollowMotor} {}

void SFDrive::ArcadeDrive(double joystickX, double joystickY)
{
  double afterLeftDeadband;
  double afterRightDeadband;
  double leftMotorOutput;
  double rightMotorOutput;

  if (fabs(joystickX) <= deadband)
    joystickX = 0;
  if (fabs(joystickY) <= deadband)
    joystickY = 0;

  double leftAbs = std::fabs(joystickX);
  double rightAbs = std::fabs(joystickY);

  // scaling

  //reason for the if statements is that if leftAbs is 0 when its under deadband, the input becomes negative, so it fixes it
  if (leftAbs != 0)
    afterLeftDeadband = (1 / (1 - deadband)) * leftAbs - (deadband / (1 / deadband));
  else
    afterLeftDeadband = 0;

  if (rightAbs != 0)
    afterRightDeadband = (1 / (1 - deadband)) * rightAbs - (deadband / (1 / deadband));
  else
    afterRightDeadband = 0;

  joystickX = std::copysign(pow(afterLeftDeadband, 2), joystickX);
  joystickY = std::copysign(pow(afterRightDeadband, 2), joystickY);

  //To fix turning backwards
  if (joystickY >= 0.0)
  {
    leftMotorOutput = joystickY + joystickX;
    rightMotorOutput = joystickY - joystickX;
  }
  else
  {
    leftMotorOutput = joystickY - joystickX;
    rightMotorOutput = joystickY + joystickX;
  }

  leftLeadMotor->Set(-leftMotorOutput);
  //negate here
  rightLeadMotor->Set(rightMotorOutput);
}

void SFDrive::PIDDrive(double positionTotal)
{
  double currentPosition = 0;
  double currentVelocity, distanceToDeccelerate, deltaTime;

  double prevTime = frc::Timer().GetFPGATimestamp();
  double wheelCicumference = M_PI * 5.7;
  double ticksPerRev = 42;

  //converting inches to ticks
  double endpoint = fabs(positionTotal) / wheelCicumference * ticksPerRev;

  //While loop, because we are not in periodic anymore
  while (endpoint > currentPosition)
  {
    double deltaTime = frc::Timer::GetFPGATimestamp() - prevTime;
    prevTime = deltaTime;
    //adding on velocity that we accelerated because of the timer
    currentVelocity = currentVelocity + (maxAcc * deltaTime);
    //currentPosition = ((1/2) * maxAcc * pow(deltaTime, 2)) + (currentVelocity * deltaTime);

    //Need to derive/check this thing again -> Do later
    //But what this is doing is checking the distance we need to deccelerate (with what's possible)
    distanceToDeccelerate = (currentVelocity * currentVelocity) / (2 * maxAcc);

    //If the amount of distance we have is less than distance to deccelerate, reduce velocity, by the most possible
    if (distanceToDeccelerate > endpoint - currentPosition)
    {
      //ask if I need some breaker here for a max decrease
      currentVelocity -= (maxAcc * deltaTime);
    }
    else
    {
      //Else increase velocity
      currentVelocity += (maxAcc * deltaTime);
      if (currentVelocity > maxVelocity)
      {
        currentVelocity = maxVelocity;
      }
      //update the current position using velocity
      //Could do encoders, but manual is pretty good
      currentPosition += currentVelocity * deltaTime;
      if(currentPosition > endpoint) {
        //this wokr?
        currentPosition = endpoint;
      }
      //they did smtng with timeout?
    }
      //but doesn't currentPosition or setPoint keep on increasing, and thus my acceleration, decceleration deosn't do anything: ask 
      leftLeadMotor->GetPIDController().SetReference(currentPosition / ticksPerRev * -1.0, rev::ControlType::kPosition, 0, 0);
      rightLeadMotor->GetPIDController().SetReference(currentPosition / ticksPerRev, rev::ControlType::kPosition, 0, 0);
  }
}