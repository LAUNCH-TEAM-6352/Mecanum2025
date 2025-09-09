// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KeyConstants;

public class Key extends SubsystemBase
{
    private static final Relay motorRelay = new Relay(KeyConstants.motorRelayChannel);

    /** Creates a new Key. */
    public Key()
    {
    }

    /** Starts the key spinning. */
    public void startSpin()
    {
        motorRelay.set(Relay.Value.kOn);
    }

    /** Stops the key spinning. */
    public void stopSpin()
    {
        motorRelay.set(Relay.Value.kOff);
    }


    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
