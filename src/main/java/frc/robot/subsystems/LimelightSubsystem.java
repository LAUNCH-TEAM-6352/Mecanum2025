package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase
{

    public LimelightSubsystem()
    {
    }

    public boolean hasTarget()
    {
        return LimelightHelpers.getTV("limelight"); // if target is visible, value of 1
    }

    public double getTargetID()
    {
        return LimelightHelpers.getFiducialID("limelight"); // no valid target ID, value of -1
    }

    public double getX()
    {
        return LimelightHelpers.getTX("limelight"); // horizontal offset in degrees
    }

    public double getY()
    {
        return LimelightHelpers.getTY("limelight"); // vertical offset in degrees
    }

    public double getDistanceToTarget(double cameraHeight, double targetHeight, double cameraPitchRadians)
    {
        if (!hasTarget())
        {
            return 0.0;
        }
        double targetAngleRadians = Math.toRadians(getY());
        /*
         * basically a rearrangement of tan(0) = opp/adj
         * - 0 is the angle from the camera to the target, considering both internal angle deviations
         * - opp is the difference in height between the camera and the target
         * - adj is the distance from the camera to the target (what we're solving for)
         */
        return (targetHeight - cameraHeight) / Math.tan(cameraPitchRadians + targetAngleRadians);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        SmartDashboard.putNumber("Limelight Target ID", getTargetID());
        SmartDashboard.putNumber("Limelight Yaw", getX()); // horizontal offset
        SmartDashboard.putNumber("Limelight Pitch", getY()); // vertical offset
        SmartDashboard.putNumber(
            "Distance to Target in meters",
            getDistanceToTarget(
                LimelightConstants.CAMERA_HEIGHT_INCHES,
                LimelightConstants.TARGET_HEIGHT_INCHES,
                LimelightConstants.CAMERA_PITCH_RADIANS));
    }
}