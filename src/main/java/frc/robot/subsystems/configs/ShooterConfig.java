package frc.robot.subsystems.configs;

import SOTAlib.Config.EncoderConfig;
import SOTAlib.Config.MotorControllerConfig;

public class ShooterConfig {
    private double p;
    private double i;
    private double d;
    private double maxLinearValue;
    private double angleConvM;
    private double angleConvB;

    private double speakerTagHeight;
    private double speakerTagToHood;
    private double limeLightHeight;
    private double limeLightAngle;
    private double limeLightToShooterPivot;
    private double pivotHeight;

    private MotorControllerConfig linearActuatorConfig;
    private EncoderConfig linearEncoderConfig;

    private MotorControllerConfig leftShooterConfig;
    private MotorControllerConfig rightShooterConfig;

    public double getSpeakerTagToHood() {
        return speakerTagToHood;
    }

    public double getLimeLightToShooterPivot() {
        return limeLightToShooterPivot;
    }

    public double getSpeakerTagHeight() {
        return speakerTagHeight;
    }

    public double getLimeLightHeight() {
        return limeLightHeight;
    }

    public double getLimeLightAngle() {
        return limeLightAngle;
    }

    public double getAngleConvM() {
        return angleConvM;
    }

    public double getAngleConvB() {
        return angleConvB;
    }

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

    public double getPivotHeight() {
        return pivotHeight;
    }
}
