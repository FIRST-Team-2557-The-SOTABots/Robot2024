package frc.robot.subsystems.configs;

import SOTAlib.Config.MotorControllerConfig;

public class ClimberConfig {
    private double climbSpeed;
    private int switchPort;
    private MotorControllerConfig motorConfig;
    private String name;

    public double getClimbSpeed() {
        return climbSpeed;
    }

    public int getSwitchPort() {
        return switchPort;
    }

    public MotorControllerConfig getMotorConfig() {
        return motorConfig;
    }

    public String getName() {
        return name;
    }
}