/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.TrajectoryDrivetrain;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static RootNamespace root = new RootNamespace("Trajectory");

    private static Supplier<Double> leftKP = root.addConstantDouble("left kP", 0);
    private static Supplier<Double> leftKI = root.addConstantDouble("left kI", 0);
    private static Supplier<Double> leftKD = root.addConstantDouble("left kD", 0);
    private static Supplier<Double> rightKP = root.addConstantDouble("right kP", 0);
    private static Supplier<Double> rightKI = root.addConstantDouble("right kI", 0);
    private static Supplier<Double> rightKD = root.addConstantDouble("right kD", 0);

    private static Supplier<Double> kV = root.addConstantDouble("kV", 0);
    private static Supplier<Double> kA = root.addConstantDouble("kA", 0);


    public static double DISTANCE_PER_PULSE = 6 * 0.0254 * Math.PI / 360;

    private static TrajectoryDrivetrain drivetrain;

    @Override
    public void robotInit() {

        SpeedControllerGroup leftSCG = new SpeedControllerGroup(
                new WPI_TalonSRX(RobotMap.LEFT_TALON), new WPI_VictorSPX(RobotMap.LEFT_VICTOR));
        SpeedControllerGroup rightSCG = new SpeedControllerGroup(
                new WPI_VictorSPX(RobotMap.RIGHT_VICTOR__1), new WPI_VictorSPX(RobotMap.RIGHT_VICTOR_2));

        Encoder leftEncoder = new Encoder(0, 1);
        Encoder rightEncoder = new Encoder(3, 2);
        ADXRS450_Gyro gyro = new ADXRS450_Gyro();

        leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        PIDSettings leftPID = new PIDSettings(leftKP, leftKI, leftKD);
        PIDSettings rightPID = new PIDSettings(rightKP, rightKI, rightKD);
        FeedForwardSettings ffs = new FeedForwardSettings(() -> 0.0, kV, kA);

        drivetrain = new TrajectoryDrivetrain(leftSCG, rightSCG, leftEncoder, rightEncoder, gyro);

        Path path = Path.of(Filesystem.getDeployDirectory().getPath(), "horizontal.wpilib.json");
        Trajectory trajectory = null;
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(path);
        } catch (IOException e) {
            System.out.println("no trajectory, this is the error");
            e.printStackTrace();
        }
        final Trajectory trajectory1 = trajectory;
        RamseteCommand ramseteCommand = FollowTrajectory.getCommand(drivetrain, trajectory, leftPID, rightPID, ffs);

        InstantCommand resetOdometry = new InstantCommand(() -> drivetrain.resetOdometry(trajectory1.getInitialPose()));
        SequentialCommandGroup followTrajectory = new SequentialCommandGroup(resetOdometry.andThen(ramseteCommand).andThen(() -> drivetrain.tankDriveVolts(0, 0)));
        root.putData("fucking move", followTrajectory);



        root.putData("reset odometry", new InstantCommand(()->
                drivetrain.resetOdometry(new Pose2d(0, 0,new Rotation2d()))));
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        root.update();
        drivetrain.periodic();
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     *
     */
    @Override
    public void autonomousInit() {

        // schedule the autonomous command (example)
    }


    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
