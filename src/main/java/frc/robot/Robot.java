// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.MoveToTarget;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    // simple proportional turning control with Limelight.
    // "proportional control" is a control algorithm in which the output is proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional to the
    // "tx" value from the Limelight.



    // *** can use getDistanceToTarget to slow down speed when distance is close to the apriltag
    
    
    
    private void drive(boolean fieldRelative)
    {

        double xSpeed = gamepad.getY();
        double ySpeed = -gamepad.getX();
        double rot = -gamepad.getRawAxis(2);

        // while the Trigger is pressed, overwrite some of the driving values with the output of our limelight methods

        if (gamepad.getRawButtonPressed(2))
        {
            new MoveToTarget(limelightSubsystem, robotDrive).schedule();
        }
    
        robotDrive.driveCartesian(xSpeed, ySpeed, rot);

    }

    
}