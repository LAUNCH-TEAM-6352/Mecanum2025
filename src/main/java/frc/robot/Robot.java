// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot 
{

    private Joystick gamepad;
    private MecanumDrive robotDrive;

    private PWMSparkMax frontLeft;
    private PWMSparkMax rearLeft;
    private PWMSparkMax frontRight;
    private PWMSparkMax rearRight;


    public void MecanumDrive()
    {
    }

    @Override
    public void robotInit() 
    {
        rearLeft = new PWMSparkMax(3);
        rearRight = new PWMSparkMax(2);
        frontRight = new PWMSparkMax(0);
        frontLeft = new PWMSparkMax(1);


        rearLeft.setInverted(true); 
        frontLeft.setInverted(true);
        

        gamepad = new Joystick(0);
        robotDrive = new MecanumDrive(frontLeft , rearLeft, frontRight, rearRight);
    }

    @Override
    public void teleopPeriodic() 
    {
        double forward = gamepad.getY();
        double strafe = -gamepad.getX();
        double rotation = -gamepad.getRawAxis(2);


        SmartDashboard.putNumber("Front Left", frontLeft.get());
        SmartDashboard.putNumber("Front Right", frontRight.get());
        SmartDashboard.putNumber("Rear Left", rearLeft.get());
        SmartDashboard.putNumber("Rear Right", rearRight.get());

        // Adjust sensitivity as needed
        forward *= 1.0;
        strafe *= 1.0;
        rotation *= 0.5;

        robotDrive.driveCartesian(forward, strafe, rotation);
    }
}