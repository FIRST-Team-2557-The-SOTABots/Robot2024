package frc.robot.subsystems.configs;

import SOTAlib.Config.EncoderConfig;
import SOTAlib.Config.MotorControllerConfig;

public class ShooterConfig {
    private double p;
    private double i;
    private double d;
    private double maxLinearValue;

    private MotorControllerConfig linearActuatorConfig;
    private EncoderConfig linearEncoderConfig;

    private MotorControllerConfig leftShooterConfig;
    private MotorControllerConfig rightShooterConfig;

    public double getMaxLinearValue() {
        return maxLinearValue;
    }

    public MotorControllerConfig getLinearActuatorConfig() {
        return linearActuatorConfig;
    }

    public EncoderConfig getLinearEncoderConfig() {
        return linearEncoderConfig;
    }

    public MotorControllerConfig getLeftShooterConfig() {
        return leftShooterConfig;
    }

    public MotorControllerConfig getRightShooterConfig() {
        return rightShooterConfig;
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
}
