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
    public static class OperatorConstants
    {
        public static final int kDriverControllerPort = 0;
    }


    public static class DriveTrainConstants
    {
        public static final int frontRightMotor = 0;
        public static final int frontLeftMotor  = 1;
        public static final int rearRightMotor = 2;
        public static final int rearLeftMotor = 3;

        public static final boolean areLeftMotorsInverted = true;

        public static class DriveTrainKeys
        {

            public static final String frontRightMotorPwr = "Front Right";
            public static final String frontLeftMotorPwr = "Front Left";
            public static final String rearRightMotorPwr = "Rear Right";
            public static final String rearLeftMotorPwr = "Rear Left";

        }
    }

    public static final class LimelightConstants
    {
        // need to be updated based on measurements
        public static final double CAMERA_HEIGHT_INCHES = 0; 
        public static final double TARGET_HEIGHT_INCHES = 2; 
        public static final double CAMERA_PITCH_RADIANS = Math.toRadians(0);
    }

}