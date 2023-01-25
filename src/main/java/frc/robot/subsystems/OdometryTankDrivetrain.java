package frc.robot.subsystems;

import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface OdometryTankDrivetrain extends Subsystem {

    void setMetersPerSecond(double leftMS, double rightMS, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                            FeedForwardSettings feedForwardSettings);

    Rotation2d getRotation2d();

    double getLeftDistance();

    double getRightDistance();

    double getLeftSpeed();

    double getRightSpeed();

    DifferentialDriveOdometry getOdometry();

    default DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
    }

    RamseteController getRamseteController();

    DifferentialDriveKinematics getKinematics();

    default Pose2d getPose() {
        return getOdometry().getPoseMeters();
    }

    default void resetOdometry(Pose2d pose2d) {
        getOdometry().resetPosition(getRotation2d(), getLeftDistance(), getRightDistance(), pose2d);
    }

    default void resetOdometry() {
        resetOdometry(new Pose2d());
    }
}
