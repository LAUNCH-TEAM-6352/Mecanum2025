// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveWithGamepad extends Command
{
    private CommandXboxController gamepad;
    private Drivetrain robotDrive;

    /** Creates a new DriveWithGamepad. */
    public DriveWithGamepad(Drivetrain robotDrive, CommandXboxController gamepad)
    {
        this.robotDrive = robotDrive;
        this.gamepad = gamepad;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(robotDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        double forward = gamepad.getLeftY();
        double strafe = -gamepad.getLeftX();
        double rotation = -gamepad.getRightY();

        // Adjust sensitivity as needed
        forward *= OperatorConstants.maxForward;
        strafe *= OperatorConstants.maxStrafe;
        rotation *= OperatorConstants.maxRotation;
        
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
