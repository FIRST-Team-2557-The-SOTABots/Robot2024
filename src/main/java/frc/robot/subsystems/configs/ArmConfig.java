package frc.robot.subsystems.configs;

import SOTAlib.Config.CompositeMotorConfig;
import SOTAlib.Config.MotorControllerConfig;

public class ArmConfig {

    private CompositeMotorConfig leftMotor;
    private MotorControllerConfig rightMotor;

    private double p;
    private double i;
    private double d;

    private double kS;
    private double kG;
    private double kV;

    public CompositeMotorConfig getLeftMotor() {
        return leftMotor;
    }

    public MotorControllerConfig getRightMotor() {
        return rightMotor;
    }

    public double getP() {
        return p;
    }

    public double getI() {
        return i;
    }

    public double getD() {
        return d;
    }

    public double getkS() {
        return kS;
    }

    public double getkG() {
        return kG;
    }

    public double getkV() {
        return kV;
    }
}
