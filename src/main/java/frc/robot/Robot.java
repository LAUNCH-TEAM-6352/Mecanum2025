// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.LimelightSubsystem;

import com.fasterxml.jackson.databind.node.NullNode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Robot extends TimedRobot
{

    private Joystick gamepad;
    private boolean autoDrive = false;
    private LimelightSubsystem limelightSubsystem;

    public MecanumDrive robotDrive;

    public PWMSparkMax rearLeft;
    public PWMSparkMax rearRight;
    public PWMSparkMax frontRight;
    public PWMSparkMax frontLeft;

    public static final double kMaxSpeed = 0.05; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI/2; // 1/2 rotation per second

    private final ADIS16470_IMU gyro = new ADIS16470_IMU();
    
    private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d rearLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d rearRightLocation = new Translation2d(-0.381, -0.381);

    private final MecanumDriveKinematics mecanumDriveKinematics = 
        new MecanumDriveKinematics(frontLeftLocation, frontRightLocation, rearLeftLocation, rearRightLocation);

   
    
    



    @Override
    public void robotInit()
    {

        gamepad = new Joystick(0);
        limelightSubsystem = new LimelightSubsystem();
      
        

        rearLeft = new PWMSparkMax(DriveTrainConstants.rearLeftMotor);
        rearRight = new PWMSparkMax(DriveTrainConstants.rearRightMotor);
        frontRight = new PWMSparkMax(DriveTrainConstants.frontRightMotor);
        frontLeft = new PWMSparkMax(DriveTrainConstants.frontLeftMotor);
        
        rearLeft.setInverted(DriveTrainConstants.areLeftMotorsInverted);
        frontLeft.setInverted(DriveTrainConstants.areLeftMotorsInverted);
        
        robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

        MecanumDriveOdometry mecanumDriveOdometry = 
        new MecanumDriveOdometry(mecanumDriveKinematics, new Rotation2d(gyro.getAngle()), 
            new MecanumDriveWheelPositions(frontLeftLocation.getNorm(), 
                                            frontRightLocation.getNorm(), 
                                            rearLeftLocation.getNorm(), 
                                            rearRightLocation.getNorm()));
    }

    @Override
    public void teleopPeriodic()
    {

        drive(true);

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
        double kP = .01;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = limelightSubsystem.getX() * kP;
        if (targetingAngularVelocity != 0) {
            System.out.println("angular velocity" + targetingAngularVelocity);
        }

        targetingAngularVelocity *= kMaxAngularSpeed;
        targetingAngularVelocity *= -1.0;


        return targetingAngularVelocity;
    }

    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are different.
    // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging
    // rather than "ty"
    double limelight_range_proportional()
    {
        double kP = 1.0;

        double targetingForwardSpeed = LimelightHelpers.getTA("limelight") * kP;
        if (targetingForwardSpeed != 0) {
            System.out.println("forward speed" + targetingForwardSpeed);
        }
        
        targetingForwardSpeed *= kMaxSpeed;
        targetingForwardSpeed *= -1.0;
        targetingForwardSpeed = 1/targetingForwardSpeed;

        return 0.1;
    }



    // *** can use getDistanceToTarget to slow down speed when distance is close to the apriltag
    private void drive(boolean fieldRelative)
    {

        double xSpeed = gamepad.getY();
        double ySpeed = -gamepad.getX();
        double rot = -gamepad.getRawAxis(2);

        // while the Trigger is pressed, overwrite some of the driving values with the output of our limelight methods
        
        if (gamepad.getTriggerPressed() && autoDrive == false)
        {
            autoDrive = true;
        }
        if (gamepad.getTriggerPressed() && autoDrive == true)
        {
            autoDrive = false;
        }
        if (autoDrive)
        {
            final double rot_limelight = limelight_aim_proportional();
            rot = rot_limelight;

            final double forward_limelight = limelight_range_proportional();
            // xSpeed = forward_limelight;
           //  ySpeed = forward_limelight / Math.tan(-theta)

            // while using Limelight, turn off field-relative driving
            fieldRelative = false;
        }

        robotDrive.driveCartesian(xSpeed, ySpeed, rot);

    }

    
}