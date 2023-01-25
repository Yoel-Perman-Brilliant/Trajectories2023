package com.spikes2212.command.drivetrains;

import com.spikes2212.path.OdometryHandler;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface OdometryDrivetrain extends Subsystem {

    public abstract OdometryHandler getHandler();

    public abstract double getWidth();

    public abstract void zeroSensors();

    public abstract double getLeftRate();

    public abstract double getRightRate();

    public abstract void setInverted(boolean inverted);

    default void updateOdometry() {
        getHandler().calculate();
    }
}
