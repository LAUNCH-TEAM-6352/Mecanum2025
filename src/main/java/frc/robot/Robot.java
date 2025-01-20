// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
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


    @Override
    public void robotInit() 
    {
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

        robotDrive.driveCartesian(forward, strafe, rotation);

    }
}