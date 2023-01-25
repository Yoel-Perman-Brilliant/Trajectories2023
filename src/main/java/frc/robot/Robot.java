// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import com.pathplanner.lib.server.PathPlannerServerThread;
import com.spikes2212.command.drivetrains.commands.DriveArcade;
import com.spikes2212.command.drivetrains.commands.DriveTank;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.DebuggableRamseteCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private CommandBase autoCommand;
    RootNamespace root = new RootNamespace("root");


    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {

        Drivetrain drivetrain = Drivetrain.getInstance();
        PathPlannerServer.startServer(5811);
        Drivetrain.getInstance().configureDashboard();

        Trajectory wpiPath = null;
        try {
            wpiPath = TrajectoryUtil.fromPathweaverJson(Path.of(Filesystem.getDeployDirectory().getPath(), "gostraight.wpilib.json"));
        } catch (IOException e) {
            e.printStackTrace();
        }


        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("dothing", new InstantCommand(() -> {throw new RuntimeException("heh?");}));
        eventMap.put("throw", new InstantCommand(() -> {throw new RuntimeException("heh? but first");}));
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("hasstop", new PathConstraints(1, 1));
        RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(drivetrain::getPose, drivetrain::resetOdometry,
                drivetrain.getRamseteController(), drivetrain.getKinematics(),
                (leftSpeed, rightSpeed) -> drivetrain.pidSet(UnifiedControlMode.VELOCITY, leftSpeed, rightSpeed, drivetrain.getLeftPIDSettings(),
                        drivetrain.getRightPIDSettings(), drivetrain.getFeedForwardSettings()), eventMap,
                false, drivetrain);
        autoCommand = autoBuilder.followPath(trajectory);
        PPRamseteCommand follow = new PPRamseteCommand(trajectory, drivetrain::getPose, drivetrain.getRamseteController(),
                drivetrain.getKinematics(),
                (leftSpeed, rightSpeed) -> drivetrain.pidSet(UnifiedControlMode.VELOCITY, leftSpeed,
                        rightSpeed, drivetrain.getLeftPIDSettings(), drivetrain.getRightPIDSettings(), drivetrain.getFeedForwardSettings()), false,
                drivetrain);
        RamseteCommand ramseteCommand = new RamseteCommand(trajectory, drivetrain::getPose, drivetrain.getRamseteController(),
                drivetrain.getKinematics(),
                (leftSpeed, rightSpeed) -> drivetrain.pidSet(UnifiedControlMode.VELOCITY, leftSpeed, rightSpeed, drivetrain.getLeftPIDSettings(),
                        drivetrain.getRightPIDSettings(), drivetrain.getFeedForwardSettings()),
                drivetrain) {

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
        };
        root.putData("wpilib ramsete command", ramseteCommand);

        root.putData("auto command", autoCommand);
        root.putData("regular follow", follow);
    }


    /**
     * This method is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        root.update();
        Drivetrain.getInstance().periodic();
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }


    /**
     * This method is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }


    @Override
    public void disabledPeriodic() {
    }


    @Override
    public void autonomousInit() {

    }


    /**
     * This method is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }


    @Override
    public void teleopInit() {
        DriveArcade driveArcade = new DriveArcade(Drivetrain.getInstance(), OI::getLeftY, OI::getRightX);
        driveArcade.schedule();
        Drivetrain.getInstance().configureDashboard();
    }


    /**
     * This method is called periodically during operator control.
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
     * This method is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
