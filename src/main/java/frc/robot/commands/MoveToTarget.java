// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LimelightSubsystem;

/** An example command that uses an example subsystem. */
public class MoveToTarget extends Command
{
    @SuppressWarnings("unused")
    private final LimelightSubsystem limelightSubsystem;
    private final MecanumDrive robotDrive;
    private final PIDController aimPIDController;
    private final PIDController rangePIDController;
    private final PIDController strafePIDController;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem
     *            The subsystem used by this command.
     */
    public MoveToTarget(LimelightSubsystem subsystem, MecanumDrive robotDrive)
    {
        this.limelightSubsystem = subsystem;
        this.robotDrive = robotDrive;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

        // Initialize PID controllers
        // aimPIDController = new PIDController(0.08, 0.035, 0.025);
        // rangePIDController = new PIDController(0.2, 0, 0);
        // strafePIDController = new PIDController(0.15, 0.05, 0.4);
        if (Math.abs(limelightSubsystem.getX()) < 8.0)
        {
            aimPIDController = new PIDController(0.04, 0.00025, 0.0); // Reduce I gain to reduce oscillation
        }
        else
        {
            aimPIDController = new PIDController(0.08, 0.0001, 0.025);
        }
        rangePIDController = new PIDController(0.1, 0.0, 0.0); // Reduce I gain to reduce oscillation
        strafePIDController = new PIDController(0.025, 0.02, 0.0); // Reduce I gain to reduce oscillation

        // Set continuous input for smoother motion
        aimPIDController.enableContinuousInput(-180, 180);
        strafePIDController.enableContinuousInput(-180, 180);

        aimPIDController.setIntegratorRange(-0.5, 0.5);

        // Set tolerances if needed
        aimPIDController.setTolerance(0.5);
        rangePIDController.setTolerance(1.0);
        strafePIDController.setTolerance(1.0);
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
        if (limelightSubsystem.hasTarget())
        {
            double currentX = limelightSubsystem.getX();
            double currentA = limelightSubsystem.getA();

            double rot = aimPIDController.calculate(currentX, 0);

            double strafe = strafePIDController.calculate(currentA, LimelightSubsystem.targetArea);
            double forward = rangePIDController.calculate(currentA, LimelightSubsystem.targetArea);

            strafe *= (currentX > 0) ? -1.0 : 1.0;

            SmartDashboard.putNumber("rotation", rot);
            SmartDashboard.putNumber("strafe", strafe);
            SmartDashboard.putNumber("forward", forward);

            robotDrive.driveCartesian(forward * -0.15, strafe * 0.15, rot * 0.3); // Adjust driving logic as needed
        }
        else
        {
            robotDrive.driveCartesian(0, 0, 0.25); // Stop the robot
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        robotDrive.driveCartesian(0, 0, 0); // Stop the robot
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}