// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.HeadConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.SpinHead;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Head;
import frc.robot.subsystems.Key;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final Drivetrain drivetrain = new Drivetrain();
    private final Head head = new Head();
    private final Key key = new Key();

    private final CommandXboxController gamepad = new CommandXboxController(OperatorConstants.driverControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();

        // Configure the dashboard:
        configureDashboard();
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
    private void configureBindings()
    {
        drivetrain.setDefaultCommand(new DriveWithGamepad(drivetrain, gamepad));

        gamepad.rightBumper().whileTrue(new SpinHead(head, DashboardConstants.headCwSpeedKey));
        gamepad.leftBumper().whileTrue(new SpinHead(head, DashboardConstants.headCcwSpeedKey));

        gamepad.x().onTrue(new InstantCommand(() -> key.startSpin(), key));
        gamepad.y().onTrue(new InstantCommand(() -> key.stopSpin(), key));
    }

    private void configureDashboard()
    {
        SmartDashboard.setDefaultNumber(DashboardConstants.headCwSpeedKey, HeadConstants.cwSpeedDefault);
        SmartDashboard.setDefaultNumber(DashboardConstants.headCcwSpeedKey, HeadConstants.ccwSpeedDefault);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return null;
    }
}