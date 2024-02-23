package frc.robot.subsystems.configs;

import SOTAlib.Config.CompositeMotorConfig;

public class ArmConfig {

    private CompositeMotorConfig leftMotor;
    private CompositeMotorConfig rightMotor;

    private double p;
    private double i;
    private double d;

    private double kS;
    private double kG;
    private double kV;

    public CompositeMotorConfig getLeftMotor() {
        return leftMotor;
    }

    public CompositeMotorConfig getRightMotor() {
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
