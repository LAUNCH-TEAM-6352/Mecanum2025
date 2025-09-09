// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static class DashboardConstants
    {
        public static final String headCwSpeedKey = "headCwSpeed";
        public static final String headCcwSpeedKey = "headCcwSpeed";
    }

    public static class DriveTrainConstants
    {
        public static final int frontRightMotorChannel = 0;
        public static final int frontLeftMotorChannel = 1;
        public static final int rearRightMotorChannel = 2;
        public static final int rearLeftMotorChannel = 3;

        public static final boolean isLeftMotorInverted = true;
        public static final boolean isRightMotorInverted = false;
    }

    public static class HeadConstants
    {
        public static final int motorChannel = 4;
        public static final boolean isMotorInverted = false;
        public static final double cwSpeedDefault = 0.6;
        public static final double ccwSpeedDefault = -0.6;
    }

    public static class KeyConstants
    {
        public static final int motorRelayChannel = 5;
    }

    public static class OperatorConstants
    {
        public static final int driverControllerPort = 0;

        // Sensitivity settings for driving
        public static final double maxForward = 0.8;
        public static final double maxStrafe = 0.8;
        public static final double maxRotation = 0.6;
    }
}