package frc.robot.subsystems.configs;

import SOTAlib.Config.CompositeMotorConfig;
import SOTAlib.Config.MotorControllerConfig;

public class WristConfig {
    private CompositeMotorConfig leftMotor;
    private MotorControllerConfig rightMotor;
    private double p;
    private double i;
    private double d;

    public double getP() {
        return p;
    }

    public double getI() {
        return i;
    }

    public double getD() {
        return d;
    }

    public CompositeMotorConfig getLeftMotor() {
        return leftMotor;
    }

    public MotorControllerConfig getRightMotor() {
        return rightMotor;
    }
}
