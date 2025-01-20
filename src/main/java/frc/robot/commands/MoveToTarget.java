// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

/** An example command that uses an example subsystem. */
public class MoveToTarget extends Command
{
    @SuppressWarnings("unused")
    private final LimelightSubsystem limelightSubsystem;
    private PIDController limeLightPID;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem
     *            The subsystem used by this command.
     */
    public MoveToTarget(LimelightSubsystem subsystem)
    {
        limelightSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

        double targetX = 1.0;

        if(subsystem.hasTarget()) {
            double id = subsystem.getTargetID();
            double y = subsystem.getY();
            double x = subsystem.getX();

            if(x < targetX){
                    
            }
        }
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