package frc.robot;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

/**
 *This is an abstract class which will be implemented to a drivetrain that will include an odometry,
 *as well as the methods required to follow a trajectory.
 *
 * For a simple example of an implementation, see
 * <a href=https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/creating-drive-subsystem.html>here</a>
 */
public abstract class OdometryDrivetrain extends TankDrivetrain {

    public OdometryDrivetrain(SpeedController left, SpeedController right) {
        super(left, right);
    }
    /**Updates the odometry.**/
    public abstract void periodic();

    /**Returns the robot's current position.**/
    public abstract Pose2d getPose();

    /**Returns the current wheel speeds of the robot.**/
    public abstract DifferentialDriveWheelSpeeds getWheelSpeeds();

    /**Returns the robot's odometry and sensors**/
    public abstract void resetOdometry(Pose2d pose);

    /**Returns a Rotation2d object based on the robot's current angle.**/
    public abstract Rotation2d rotFromAngle();

    /**Sets the speed controllers' voltage.
     @param leftVolts the left speed controller's voltage
     @param rightVolts the right speed controller's voltage**/

    public abstract void tankDriveVolts(double leftVolts, double rightVolts);

    /**Returns the robot's width.**/
    public abstract double getWidth();



}
