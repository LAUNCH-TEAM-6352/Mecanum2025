// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.DriveTrainConstants.DriveTrainKeys;

public class Drivetrain extends SubsystemBase
{

  public MecanumDrive mecanumDrive;

  public PWMSparkMax rearLeft;
  public PWMSparkMax rearRight;
  public PWMSparkMax frontRight;
  public PWMSparkMax frontLeft;

  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  /** Creates a new Drivetrain. */
  public Drivetrain()
  {
    rearLeft = new PWMSparkMax(DriveTrainConstants.rearLeftMotor);
    rearRight = new PWMSparkMax(DriveTrainConstants.rearRightMotor);
    frontRight = new PWMSparkMax(DriveTrainConstants.frontRightMotor);
    frontLeft = new PWMSparkMax(DriveTrainConstants.frontLeftMotor);

    rearLeft.setInverted(DriveTrainConstants.areLeftMotorsInverted);
    frontLeft.setInverted(DriveTrainConstants.areLeftMotorsInverted);

  }

  public void drive(double forward, double strafe, double rotation)
  {

    mecanumDrive.driveCartesian(forward, strafe, rotation);

  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(DriveTrainKeys.frontLeftMotorPwr, frontLeft.get());
    SmartDashboard.putNumber(DriveTrainKeys.frontRightMotorPwr, frontRight.get());
    SmartDashboard.putNumber(DriveTrainKeys.rearLeftMotorPwr, rearLeft.get());
    SmartDashboard.putNumber(DriveTrainKeys.rearRightMotorPwr, rearRight.get());
  }

}
