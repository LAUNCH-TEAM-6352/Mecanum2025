// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  
  PWMSparkMax rearLeft = new PWMSparkMax(Constants.DriveTrainConstants.rearLeftMotor);
  PWMSparkMax rearRight = new PWMSparkMax(Constants.DriveTrainConstants.rearRightMotor);
  PWMSparkMax frontRight = new PWMSparkMax(Constants.DriveTrainConstants.frontRightMotor);
  PWMSparkMax frontLeft = new PWMSparkMax(Constants.DriveTrainConstants.frontLeftMotor);

  /** Creates a new Drivetrain. */
  public Drivetrain() 
  {
  
    

  }

  public void drive(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower)
  {
    // Normalize motor powers
    double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

    if (maxPower > 1) {
        frontLeftPower /= maxPower;
        frontRightPower /= maxPower;
        backLeftPower /= maxPower;
        backRightPower /= maxPower;
    }

    // Set motor powers
    frontLeft.set(frontLeftPower);
    frontRight.set(frontRightPower);
    rearLeft.set(backLeftPower);
    rearRight.set(backRightPower);
  }

  public void reverse(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower)
  {
    // Normalize motor powers
    double minPower = Math.min(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

    if (minPower < 1) {
        frontLeftPower /= minPower;
        frontRightPower /= minPower;
        backLeftPower /= minPower;
        backRightPower /= minPower;
    }

    // Set motor powers
    frontLeft.set(frontLeftPower);
    frontRight.set(frontRightPower);
    rearLeft.set(backLeftPower);
    rearRight.set(backRightPower);
  }
  
  public void strafeRight(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower)
  {
    // Normalize motor powers
    double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

    if (maxPower < 1) {
        frontLeftPower /= maxPower;
        frontRightPower /= maxPower;
        backLeftPower /= maxPower;
        backRightPower /= maxPower;
    }

    // Set motor powers
    frontLeft.set(frontLeftPower);
    frontRight.set(-frontRightPower);
    rearLeft.set(-backLeftPower);
    rearRight.set(backRightPower);
  }  
  
  public void strafeLeft(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower)
  {
      // Normalize motor powers
    double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

    if (maxPower < 1) {
        frontLeftPower /= maxPower;
        frontRightPower /= maxPower;
        backLeftPower /= maxPower;
        backRightPower /= maxPower;
    }

    // Set motor powers
    frontLeft.set(-frontLeftPower);
    frontRight.set(frontRightPower);
    rearLeft.set(backLeftPower);
    rearRight.set(-backRightPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
