// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
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
        // P = 0.05
        // I = 0.01
        // D = 0.02
        aimPIDController = new PIDController(0.05, 0.02, 0.025);
        rangePIDController = new PIDController(0.1, 0.01, 0);

        // Set tolerances if needed
        aimPIDController.setTolerance(1.0);
        rangePIDController.setTolerance(1.0);
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
            double rot = aimPIDController.calculate(limelightSubsystem.getX(), 0);
            double strafe = rangePIDController.calculate(limelightSubsystem.getA(), LimelightSubsystem.targetArea);

            System.out.println(rot);
            double forward = rangePIDController.calculate(limelightSubsystem.getA(), LimelightSubsystem.targetArea);
            // double strafe = 0.30;
            if (rot > 0) {
                strafe *= -1.0;
            }
            // strafe *= (limelightSubsystem.getX() < 0) ? -1.0 : 1.0;
            System.out.println("rot" + rot);
            System.out.println("strafe" + strafe);
            robotDrive.driveCartesian(forward * -0.15, strafe * 0.15, rot * 0.3); // Adjust driving logic as needed
        }
        else
        {
            robotDrive.driveCartesian(0, 0, 0); // Stop the robot if no target is found
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