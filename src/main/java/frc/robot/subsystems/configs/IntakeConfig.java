package frc.robot.subsystems.configs;

import SOTAlib.Config.MotorControllerConfig;

public class IntakeConfig {
    private double intakeVoltage;
    private MotorControllerConfig motorConfig;
    private int sensorTrigger;

    public MotorControllerConfig getMotorConfig() {
        return motorConfig;
    }

    public double getIntakeVoltage() {
        return intakeVoltage;
    }

    public int getSensorTrigger() {
        return sensorTrigger;
    }
}
