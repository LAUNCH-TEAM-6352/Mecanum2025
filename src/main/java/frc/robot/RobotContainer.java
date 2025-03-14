// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.LimelightConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private final Drivetrain drivetrain = new Drivetrain();
    private final CommandXboxController gamepad = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        limelightSubsystem.setDefaultCommand(new RunCommand(() -> {
            if (limelightSubsystem.hasTarget()) {
                System.out.println("Target ID: " + limelightSubsystem.getTargetID());
                double distance = limelightSubsystem.getDistanceToTarget(
                        LimelightConstants.CAMERA_HEIGHT_INCHES, 
                        LimelightConstants.TARGET_HEIGHT_INCHES, 
                        LimelightConstants.CAMERA_PITCH_RADIANS);
                System.out.println("Distance to target: " + distance);
            } else {
                System.out.println("No target detected.");
            }
        }, limelightSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new RunCommand(() -> {
            if (limelightSubsystem.hasTarget()) {
                double distance = limelightSubsystem.getDistanceToTarget(
                        LimelightConstants.CAMERA_HEIGHT_INCHES, 
                        LimelightConstants.TARGET_HEIGHT_INCHES, 
                        LimelightConstants.CAMERA_PITCH_RADIANS);
                System.out.println("Autonomous Target Distance: " + distance);
            }
        });
    }
}