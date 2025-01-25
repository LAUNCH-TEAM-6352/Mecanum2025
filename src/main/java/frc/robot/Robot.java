// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Robot extends TimedRobot
{

    private Joystick gamepad;
    private boolean autoDrive = false;

    public MecanumDrive robotDrive;

    public PWMSparkMax rearLeft;
    public PWMSparkMax rearRight;
    public PWMSparkMax frontRight;
    public PWMSparkMax frontLeft;

    public static final double kMaxSpeed = 5.0; // 3 meters per second
    public static final double kMaxAngularSpeed = 2*Math.PI; // 1/2 rotation per second

    @Override
    public void robotInit()
    {

        gamepad = new Joystick(0);
        
        rearLeft = new PWMSparkMax(DriveTrainConstants.rearLeftMotor);
        rearRight = new PWMSparkMax(DriveTrainConstants.rearRightMotor);
        frontRight = new PWMSparkMax(DriveTrainConstants.frontRightMotor);
        frontLeft = new PWMSparkMax(DriveTrainConstants.frontLeftMotor);
        
        rearLeft.setInverted(DriveTrainConstants.areLeftMotorsInverted);
        frontLeft.setInverted(DriveTrainConstants.areLeftMotorsInverted);
        
        robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    }

    @Override
    public void teleopPeriodic()
    {

        drive(true);

    }

    public void drive(double forward, double strafe, double rotation)
    {
  
      robotDrive.driveCartesian(forward, strafe, rotation);
  
    }

    private void drive(boolean fieldRelative)
    {

        double xSpeed = gamepad.getY();
        double ySpeed = -gamepad.getX();
        double rot = -gamepad.getRawAxis(2);

        // while the Trigger is pressed, overwrite some of the driving values with the output of our limelight methods
        if (gamepad.getTrigger() && autoDrive == false)
        {
            autoDrive = true;
        }
        if (gamepad.getTrigger() && autoDrive == true)
        {
            autoDrive = false;
        }
        if (autoDrive)
        {
            final double rot_limelight = limelight_aim_proportional();
            rot = rot_limelight;

            final double forward_limelight = limelight_range_proportional();
            xSpeed = forward_limelight;

            // while using Limelight, turn off field-relative driving.
            fieldRelative = false;
        }

        drive(xSpeed, ySpeed, rot);
    }

    // simple proportional turning control with Limelight.
    // "proportional control" is a control algorithm in which the output is proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional to the
    // "tx" value from the Limelight.
    double limelight_aim_proportional()
    {
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .035;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = -LimelightHelpers.getTX("limelight") * kP;

        targetingAngularVelocity *= kMaxAngularSpeed;

        return targetingAngularVelocity;
    }

    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are different.
    // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging
    // rather than "ty"
    double limelight_range_proportional()
    {
        double kP = .1;

        double targetingForwardSpeed = -LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= kMaxSpeed;

        return targetingForwardSpeed;
    }

}