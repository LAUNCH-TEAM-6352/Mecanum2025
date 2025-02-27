// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase
{

  public MecanumDrive mecanumDrive;

  private final PWMSparkMax rearLeft = new PWMSparkMax(Constants.DriveTrainConstants.rearLeftMotorChannel);
  private final PWMSparkMax rearRight = new PWMSparkMax(Constants.DriveTrainConstants.rearRightMotorChannel);
  private final PWMSparkMax frontRight = new PWMSparkMax(Constants.DriveTrainConstants.frontRightMotorChannel);
  private final PWMSparkMax frontLeft = new PWMSparkMax(Constants.DriveTrainConstants.frontLeftMotorChannel);

  /** Creates a new Drivetrain. */
  public Drivetrain()
  {
    mecanumDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
  }

  public void drive(double forward, double strafe, double rotate)
  {
    mecanumDrive.driveCartesian(forward, strafe, rotate);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }
}
