package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.spikes2212.command.drivetrains.OdometryDrivetrain;
import com.spikes2212.command.drivetrains.SparkMaxDrivetrain;
import com.spikes2212.command.drivetrains.commands.MoveSmartMotorControllerTankDrivetrain;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.dashboard.Namespace;
import com.spikes2212.path.OdometryHandler;
import com.spikes2212.util.PigeonWrapper;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotMap;
import frc.robot.commands.DebuggableRamseteCommand;
import frc.robot.commands.FollowTrajectory;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.function.Supplier;

public class Drivetrain extends SparkMaxDrivetrain implements OdometryTankDrivetrain, OdometryDrivetrain {

    public static final double WHEEL_DIAMETER_IN_INCHES = 6;
    public static final double INCHES_TO_CM = 2.54;
    public static final double GEAR_RATIO = 1 / 11.161;
    public static final double DISTANCE_PER_PULSE = WHEEL_DIAMETER_IN_INCHES * INCHES_TO_CM * GEAR_RATIO * Math.PI / 100;

    private static final double WIDTH = 0.58;

    private final Namespace pidNamespace = namespace.addChild("pid");
    private final Supplier<Double> kPLeft = pidNamespace.addConstantDouble("left kP", 0);
    private final Supplier<Double> kILeft = pidNamespace.addConstantDouble("left kI", 0);
    private final Supplier<Double> kDLeft = pidNamespace.addConstantDouble("left kD", 0);
    private final Supplier<Double> toleranceLeft = pidNamespace.addConstantDouble("left tolerance", 0);
    private final Supplier<Double> waitTimeLeft = pidNamespace.addConstantDouble("left wait time", 0);
    private final PIDSettings leftPIDSettings = new PIDSettings(kPLeft, kILeft, kDLeft, toleranceLeft, waitTimeLeft);

    private final Supplier<Double> kPRight = pidNamespace.addConstantDouble("right kP", 0);
    private final Supplier<Double> kIRight = pidNamespace.addConstantDouble("right kI", 0);
    private final Supplier<Double> kDRight = pidNamespace.addConstantDouble("right kD", 0);
    private final Supplier<Double> toleranceRight = pidNamespace.addConstantDouble("right tolerance", 0);
    private final Supplier<Double> waitTimeRight = pidNamespace.addConstantDouble("right time", 0);
    private final PIDSettings rightPIDSettings = new PIDSettings(kPRight, kIRight, kDRight, toleranceRight, waitTimeRight);


    private final Supplier<Double> kS = pidNamespace.addConstantDouble("kS", 0);
    private final Supplier<Double> kV = pidNamespace.addConstantDouble("kV", 0);
    private final Supplier<Double> kA = pidNamespace.addConstantDouble("kA", 0);
    private final FeedForwardSettings feedForwardSettings = new FeedForwardSettings(kS, kV, kA);

    private final Supplier<Double> leftSetpoint = namespace.addConstantDouble("left setpoint", 1);
    private final Supplier<Double> rightSetpoint = namespace.addConstantDouble("right setpoint", 1);

    private final ADXRS450_Gyro gyro;

    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics;
    private final RamseteController ramseteController;

    private final OdometryHandler handler;

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain(
                    "drivetrain",
                    new CANSparkMax(RobotMap.CAN.DRIVETRAIN_LEFT_SPARKMAX_2, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.DRIVETRAIN_RIGHT_SPARKMAX_2, CANSparkMaxLowLevel.MotorType.kBrushless),
                    List.of(new CANSparkMax(RobotMap.CAN.DRIVETRAIN_LEFT_SPARKMAX_1, CANSparkMaxLowLevel.MotorType.kBrushless)),
                    List.of(new CANSparkMax(RobotMap.CAN.DRIVETRAIN_RIGHT_SPARKMAX_1, CANSparkMaxLowLevel.MotorType.kBrushless)),
                    WIDTH

            );
        }
        return instance;
    }

    public Drivetrain(String namespaceName, CANSparkMax leftMaster, CANSparkMax rightMaster,
                      List<CANSparkMax> leftSlaves, List<CANSparkMax> rightSlaves, double width) {
        super(namespaceName, leftMaster, rightMaster, leftSlaves, rightSlaves);
        leftMaster.getEncoder().setPositionConversionFactor(DISTANCE_PER_PULSE);
        rightMaster.getEncoder().setPositionConversionFactor(DISTANCE_PER_PULSE);
        leftMaster.getEncoder().setVelocityConversionFactor(DISTANCE_PER_PULSE/60);
        rightMaster.getEncoder().setVelocityConversionFactor(DISTANCE_PER_PULSE/60);
        gyro = new ADXRS450_Gyro();
        odometry = new DifferentialDriveOdometry(getRotation2d(), getLeftDistance(), getRightDistance());
        kinematics = new DifferentialDriveKinematics(width);
        ramseteController = new RamseteController();
        handler = new OdometryHandler(this::getLeftDistance, this::getRightDistance, gyro::getAngle, 0, 0);

    }

    @Override
    public void configureLoop(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings, FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        super.configureLoop(leftPIDSettings, rightPIDSettings, feedForwardSettings, trapezoidProfileSettings);
        rightMaster.setInverted(true);
        leftMaster.getEncoder().setPositionConversionFactor(DISTANCE_PER_PULSE);
        rightMaster.getEncoder().setPositionConversionFactor(DISTANCE_PER_PULSE);
        leftMaster.getEncoder().setVelocityConversionFactor(DISTANCE_PER_PULSE/60);
        rightMaster.getEncoder().setVelocityConversionFactor(DISTANCE_PER_PULSE/60);
        leftMaster.getPIDController().setOutputRange(-1,1);
    }

    public PIDSettings getLeftPIDSettings() {
        return leftPIDSettings;
    }

    public PIDSettings getRightPIDSettings() {
        return rightPIDSettings;
    }

    public FeedForwardSettings getFeedForwardSettings() {
        return feedForwardSettings;
    }

    @Override
    public void configureDashboard() {
        namespace.putData("move", new MoveSmartMotorControllerTankDrivetrain(this, leftPIDSettings, rightPIDSettings,
                feedForwardSettings, UnifiedControlMode.VELOCITY, leftSetpoint, rightSetpoint));
        namespace.putNumber("left position", leftMaster.getEncoder()::getPosition);
        namespace.putNumber("right position", rightMaster.getEncoder()::getPosition);
        namespace.putNumber("left speed1", leftMaster.getEncoder()::getVelocity);
        namespace.putNumber("right speed1", rightMaster.getEncoder()::getVelocity);
        namespace.putNumber("left velocity conversion factor", leftMaster.getEncoder()::getVelocityConversionFactor);
        namespace.putNumber("right velocity conversion factor", rightMaster.getEncoder()::getVelocityConversionFactor);
        namespace.putBoolean("left master inverted", leftMaster::getInverted);
        namespace.putBoolean("left slave inverted", leftSlaves.get(0)::getInverted);
        namespace.putBoolean("right master inverted", rightMaster::getInverted);
        namespace.putBoolean("right slave inverted", rightSlaves.get(0)::getInverted);
        namespace.putNumber("left ff", () -> leftMaster.getPIDController().getFF());
        namespace.putData("change left ff", new InstantCommand(() -> leftMaster.getPIDController().setFF(0.6)));
        namespace.putData("reset odometry", new InstantCommand(this::resetOdometry));
        namespace.putData("reset devices", new InstantCommand(() -> {
            leftMaster.getEncoder().setPosition(0);
            rightMaster.getEncoder().setPosition(0);
            gyro.reset();
        }));

        namespace.putNumber("x", () -> odometry.getPoseMeters().getX());
        namespace.putNumber("y", () -> odometry.getPoseMeters().getY());
        namespace.putNumber("rotation 2d", () -> odometry.getPoseMeters().getRotation().getDegrees());


    }

    @Override
    public void setMetersPerSecond(double leftMS, double rightMS, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                                   FeedForwardSettings feedForwardSettings) {
        pidSet(UnifiedControlMode.VELOCITY, leftMS, rightMS, leftPIDSettings, rightPIDSettings, feedForwardSettings);
    }

    @Override
    public void periodic() {
        super.periodic();
        odometry.update(getRotation2d(), getLeftDistance(), getRightDistance());
    }

    @Override
    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    @Override
    public double getLeftDistance() {
        return leftMaster.getEncoder().getPosition();
    }

    @Override
    public double getRightDistance() {
        return rightMaster.getEncoder().getPosition();
    }

    @Override
    public double getLeftSpeed() {
        return leftMaster.getEncoder().getVelocity();
    }

    @Override
    public double getRightSpeed() {
        return rightMaster.getEncoder().getVelocity();
    }

    @Override
    public DifferentialDriveOdometry getOdometry() {
        return odometry;
    }

    @Override
    public RamseteController getRamseteController() {
        return ramseteController;
    }

    @Override
    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }


    @Override
    public OdometryHandler getHandler() {
        return handler;
    }

    @Override
    public double getWidth() {
        return WIDTH;
    }

    @Override
    public void zeroSensors() {
        leftMaster.getEncoder().setPosition(0);
        rightMaster.getEncoder().setPosition(0);
        gyro.reset();
    }

    @Override
    public double getLeftRate() {
        return leftMaster.getEncoder().getVelocity();
    }

    @Override
    public double getRightRate() {
        return rightMaster.getEncoder().getVelocity();
    }

    @Override
    public void setInverted(boolean inverted) {
        leftMaster.setInverted(inverted);
        rightMaster.setInverted(!inverted);
    }
}
