// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveWithGamepad extends Command
{

  private Joystick gamepad;
  private Drivetrain robotDrive;

  private PWMSparkMax frontLeft;
  private PWMSparkMax rearLeft;
  private PWMSparkMax frontRight;
  private PWMSparkMax rearRight;

  /** Creates a new DriveWithGamepad. */
  public DriveWithGamepad(Drivetrain robotDrive)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    rearLeft.setInverted(DriveTrainConstants.isLeftMotorInverted);
    frontLeft.setInverted(DriveTrainConstants.isRightMotorInverted);

    gamepad = new Joystick(0);
    robotDrive = new Drivetrain();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    double forward = gamepad.getY();
    double strafe = -gamepad.getX();
    double rotation = -gamepad.getRawAxis(2);

    SmartDashboard.putNumber("Front Left", frontLeft.get());
    SmartDashboard.putNumber("Front Right", frontRight.get());
    SmartDashboard.putNumber("Rear Left", rearLeft.get());
    SmartDashboard.putNumber("Rear Right", rearRight.get());

    robotDrive.drive(forward, strafe, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
