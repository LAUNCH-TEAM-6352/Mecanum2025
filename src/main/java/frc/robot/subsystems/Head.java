// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HeadConstants;

public class Head extends SubsystemBase
{
    private static final PWMSparkMax motor = new PWMSparkMax(HeadConstants.motorChannel);

    /** Creates a new Head. */
    public Head()
    {
        motor.setInverted(HeadConstants.isMotorInverted);
    }

    /**
     * Spin the head.
     */
    public void spin(double speed)
    {
        motor.set(speed);
    }

    /**
     * Stop the head.
     */
    public void stopSpin()
    {
        spin(0.0);
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
