package frc.robot;

import com.ctre.phoenix.motorcontrol.IFollower;
import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class OI {

    private static final PlaystationControllerWrapper ps = new PlaystationControllerWrapper(0);

    public static double getLeftX() {
        return -ps.getLeftX();
    }

    public static double getLeftY() {
        return -ps.getLeftY();
    }

    public static double getRightX() {
        return -ps.getRightX();
    }

    public static double getRightY() {
        return -ps.getRightY();
    }

    public static void initialize() {
    }
}
