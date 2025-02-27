// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class MoveToTarget extends Command
{
    @SuppressWarnings("unused")
    private final LimelightSubsystem limelight;
    private final Drivetrain drivetrain;
    private final PIDController strafePID, forwardPID, rotationPID;

    public MoveToTarget(LimelightSubsystem limelight, Drivetrain drivetrain)
    {
        this.limelight = limelight;
        this.drivetrain = drivetrain;

        addRequirements(limelight, drivetrain);

        strafePID = new PIDController(DriveTrainConstants.PIDConstants.strafekP,
                                     DriveTrainConstants.PIDConstants.strafekI,
                                     DriveTrainConstants.PIDConstants.strafekD);

        forwardPID = new PIDController(DriveTrainConstants.PIDConstants.forwardkP,
                                      DriveTrainConstants.PIDConstants.forwardkI,
                                      DriveTrainConstants.PIDConstants.forwardkD);

        rotationPID = new PIDController(DriveTrainConstants.PIDConstants.rotkP,
                                       DriveTrainConstants.PIDConstants.rotkI,
                                       DriveTrainConstants.PIDConstants.rotkD);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        strafePID.reset();
        forwardPID.reset();
        rotationPID.reset();

        Pose2d botPose = limelight.getPose2d();

        drivetrain.drive(forwardPID.calculate(botPose.getY(), 0.0),
                         strafePID.calculate(botPose.getX(), 0.0),
                         rotationPID.calculate(botPose.getRotation().getDegrees(), 0.0));
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
        return limelight.atTarget();
    }
}