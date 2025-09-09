// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot 
{
    @SuppressWarnings("unused")
    private final RobotContainer robotContainer;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public Robot()
    {
        // Instantiate our RobotContainer. This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic()
    {
        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods. This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }
    
    @Override
    public void disabledPeriodic()
    {
    }

    @Override
    public void robotInit() 
    {
    
    }

    @Override
    public void teleopPeriodic() 
    {

    }
}