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
        public static final int frontRightMotorChannel = 0;
        public static final int frontLeftMotorChannel  = 1;
        public static final int rearRightMotorChannel = 2;
        public static final int rearLeftMotorChannel = 3;

        public static final double maxSpeed = 5.0;

        public static final boolean isLeftMotorInverted = true;
        public static final boolean isRightMotorInverted = true;

        public static class PIDConstants
        {
            public static final double strafekP = 0.0;
            public static final double strafekI = 0.0;
            public static final double strafekD = 0.0;
            public static final double strafeTol = 0.0;

            public static final double forwardkP = 0.0;
            public static final double forwardkI = 0.0;
            public static final double forwardkD = 0.0;
            public static final double forwardTol = 0.0;

            public static final double rotkP = 0.0;
            public static final double rotkI = 0.0;
            public static final double rotkD = 0.0;
            public static final double rotTol = 0.0;

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