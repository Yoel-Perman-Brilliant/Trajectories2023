// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

/**
 * A command that uses a RAMSETE controller ({@link RamseteController}) to follow a trajectory
 * {@link Trajectory} with a differential drive.
 *
 * <p>The command handles trajectory-following, PID calculations, and feedforwards internally. This
 * is intended to be a more-or-less "complete solution" that can be used by teams without a great
 * deal of controls expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to use the onboard PID
 * functionality of a "smart" motor controller) may use the secondary constructor that omits the PID
 * and feedforward functionality, returning only the raw wheel speeds from the RAMSETE controller.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class DebuggableRamseteCommand extends CommandBase {

    private final RootNamespace root = new RootNamespace("ramsete command");

    protected final Timer timer = new Timer();
    protected final boolean usePID;
    protected final Trajectory trajectory;
    protected final Supplier<Pose2d> pose;
    protected final RamseteController follower;
    protected final SimpleMotorFeedforward feedforward;
    protected final DifferentialDriveKinematics kinematics;
    protected final Supplier<DifferentialDriveWheelSpeeds> speeds;
    protected final PIDController leftController;
    protected final PIDController rightController;
    protected final BiConsumer<Double, Double> output;
    protected DifferentialDriveWheelSpeeds prevSpeeds;
    protected double prevTime;

    /**
     * Constructs a new DebuggableRamseteCommand that, when executed, will follow the provided trajectory. PID
     * control and feedforward are handled internally, and outputs are scaled -12 to 12 representing
     * units of volts.
     *
     * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
     * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
     *
     * @param trajectory The trajectory to follow.
     * @param pose A function that supplies the robot pose - use one of the odometry classes to
     *     provide this.
     * @param controller The RAMSETE controller used to follow the trajectory.
     * @param feedforward The feedforward to use for the drive.
     * @param kinematics The kinematics for the robot drivetrain.
     * @param wheelSpeeds A function that supplies the speeds of the left and right sides of the robot
     *     drive.
     * @param leftController The PIDController for the left side of the robot drive.
     * @param rightController The PIDController for the right side of the robot drive.
     * @param outputVolts A function that consumes the computed left and right outputs (in volts) for
     *     the robot drive.
     * @param requirements The subsystems to require.
     */
    public DebuggableRamseteCommand(
            Trajectory trajectory,
            Supplier<Pose2d> pose,
            RamseteController controller,
            SimpleMotorFeedforward feedforward,
            DifferentialDriveKinematics kinematics,
            Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
            PIDController leftController,
            PIDController rightController,
            BiConsumer<Double, Double> outputVolts,
            Subsystem... requirements) {
        this.trajectory = requireNonNullParam(trajectory, "trajectory", "DebuggableRamseteCommand");
        this.pose = requireNonNullParam(pose, "pose", "DebuggableRamseteCommand");
        follower = requireNonNullParam(controller, "controller", "DebuggableRamseteCommand");
        this.feedforward = feedforward;
        this.kinematics = requireNonNullParam(kinematics, "kinematics", "DebuggableRamseteCommand");
        speeds = requireNonNullParam(wheelSpeeds, "wheelSpeeds", "DebuggableRamseteCommand");
        this.leftController = requireNonNullParam(leftController, "leftController", "DebuggableRamseteCommand");
        this.rightController = requireNonNullParam(rightController, "rightController", "DebuggableRamseteCommand");
        output = requireNonNullParam(outputVolts, "outputVolts", "DebuggableRamseteCommand");

        usePID = true;

        addRequirements(requirements);
    }

    /**
     * Constructs a new DebuggableRamseteCommand that, when executed, will follow the provided trajectory.
     * Performs no PID control and calculates no feedforwards; outputs are the raw wheel speeds from
     * the RAMSETE controller, and will need to be converted into a usable form by the user.
     *
     * @param trajectory The trajectory to follow.
     * @param pose A function that supplies the robot pose - use one of the odometry classes to
     *     provide this.
     * @param follower The RAMSETE follower used to follow the trajectory.
     * @param kinematics The kinematics for the robot drivetrain.
     * @param outputMetersPerSecond A function that consumes the computed left and right wheel speeds.
     * @param requirements The subsystems to require.
     */
    public DebuggableRamseteCommand(
            Trajectory trajectory,
            Supplier<Pose2d> pose,
            RamseteController follower,
            DifferentialDriveKinematics kinematics,
            BiConsumer<Double, Double> outputMetersPerSecond,
            Subsystem... requirements) {
        this.trajectory = requireNonNullParam(trajectory, "trajectory", "DebuggableRamseteCommand");
        this.pose = requireNonNullParam(pose, "pose", "DebuggableRamseteCommand");
        this.follower = requireNonNullParam(follower, "follower", "DebuggableRamseteCommand");
        this.kinematics = requireNonNullParam(kinematics, "kinematics", "DebuggableRamseteCommand");
        output =
                requireNonNullParam(outputMetersPerSecond, "outputMetersPerSecond", "DebuggableRamseteCommand");

        feedforward = null;
        speeds = null;
        leftController = null;
        rightController = null;

        usePID = false;

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        prevTime = -1;
        var initialState = trajectory.sample(0);
        prevSpeeds =
                kinematics.toWheelSpeeds(
                        new ChassisSpeeds(
                                initialState.velocityMetersPerSecond,
                                0,
                                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
        timer.reset();
        timer.start();
        if (usePID) {
            leftController.reset();
            rightController.reset();
        }
    }

    @Override
    public void execute() {
        double curTime = timer.get();
        double dt = curTime - prevTime;

        if (prevTime < 0) {
            output.accept(0.0, 0.0);
            prevTime = curTime;
            return;
        }

        var targetWheelSpeeds =
                kinematics.toWheelSpeeds(
                        follower.calculate(pose.get(), trajectory.sample(curTime)));

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        root.putNumber("left setpoint", leftSpeedSetpoint);
        root.putNumber("right setpoint", rightSpeedSetpoint);

        double leftOutput;
        double rightOutput;

        if (usePID) {
            double leftFeedforward =
                    feedforward.calculate(
                            leftSpeedSetpoint, (leftSpeedSetpoint - prevSpeeds.leftMetersPerSecond) / dt);

            double rightFeedforward =
                    feedforward.calculate(
                            rightSpeedSetpoint, (rightSpeedSetpoint - prevSpeeds.rightMetersPerSecond) / dt);

            leftOutput =
                    leftFeedforward
                            + leftController.calculate(speeds.get().leftMetersPerSecond, leftSpeedSetpoint);

            rightOutput =
                    rightFeedforward
                            + rightController.calculate(
                            speeds.get().rightMetersPerSecond, rightSpeedSetpoint);
        } else {
            leftOutput = leftSpeedSetpoint;
            rightOutput = rightSpeedSetpoint;
        }

        output.accept(leftOutput, rightOutput);
        prevSpeeds = targetWheelSpeeds;
        prevTime = curTime;

        if (speeds != null) {
            root.putNumber("left difference", leftOutput - speeds.get().leftMetersPerSecond);
            root.putNumber("right difference", rightOutput - speeds.get().rightMetersPerSecond);
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        if (interrupted) {
            output.accept(0.0, 0.0);
        }
    }

    @Override
    public boolean isFinished() {
//        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
        return false;
    }

    public double getTime() {
        return timer.get();
    }

//    @Override
//    public void initSendable(SendableBuilder builder) {
//        super.initSendable(builder);
//        builder.addDoubleProperty("leftVelocity", () -> prevSpeeds.leftMetersPerSecond, null);
//        builder.addDoubleProperty("rightVelocity", () -> prevSpeeds.rightMetersPerSecond, null);
//    }
}
