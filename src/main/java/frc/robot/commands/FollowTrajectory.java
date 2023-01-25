package frc.robot.commands;

import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.OdometryTankDrivetrain;

public class FollowTrajectory extends DebuggableRamseteCommand {

    private final RootNamespace rootNamespace = new RootNamespace("follow trajectory");

    private final Timer timer;

    public FollowTrajectory(OdometryTankDrivetrain drivetrain, Trajectory trajectory, FeedForwardSettings feedForwardSettings,
                            PIDSettings leftPIDSettings, PIDSettings rightPIDSettings) {
        super(trajectory, drivetrain::getPose, drivetrain.getRamseteController(), drivetrain.getKinematics(),
                (leftMS, rightMS) -> drivetrain.setMetersPerSecond(leftMS, rightMS, leftPIDSettings,
                        rightPIDSettings, feedForwardSettings), drivetrain);
        timer = new Timer();

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Command");
        builder.addStringProperty(".name", this::getName, null);
        builder.addBooleanProperty(
                "running",
                this::isScheduled,
                value -> {
                    if (value) {
                        if (!isScheduled()) {
                            schedule();
                        }
                    } else {
                        if (isScheduled()) {
                            cancel();
                        }
                    }
                });
        builder.addBooleanProperty(
                ".isParented", () -> CommandScheduler.getInstance().isComposed(this), null);
        builder.addStringProperty(
                "interruptBehavior", () -> getInterruptionBehavior().toString(), null);
        builder.addBooleanProperty("runsWhenDisabled", this::runsWhenDisabled, null);
    }

    @Override
    public void initialize() {
        super.initialize();
        timer.start();
    }

    public double getTime() {
        return timer.get();
    }
}
