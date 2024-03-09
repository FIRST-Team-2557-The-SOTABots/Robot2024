package frc.robot.subsystems.configs;

import SOTAlib.Config.CompositeMotorConfig;
import SOTAlib.Config.MotorControllerConfig;

public class WristConfig {
    private CompositeMotorConfig leftMotor;
    private MotorControllerConfig rightMotor;
    private double p;
    private double i;
    private double d;
    private double iZone;
    private double minOutputRange;
    private double maxOutputRange;
    private int leftMotorPort;
    private int rightMotorPort;
    private boolean rightMotorInverted;
    private boolean leftMotorInverted;

    

    public double getMinOutputRange() {
        return minOutputRange;
    }


    public int getLeftMotorPort() {
        return leftMotorPort;
    }

    public int getRightMotorPort() {
        return rightMotorPort;
    }

    public boolean isRightMotorInverted() {
        return rightMotorInverted;
    }

    public boolean isLeftMotorInverted() {
        return leftMotorInverted;
    }

    public double getiZone() {
        return iZone;
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

    public CompositeMotorConfig getLeftMotor() {
        return leftMotor;
    }

    public MotorControllerConfig getRightMotor() {
        return rightMotor;
    }


    public double getMaxOutputRange() {
        return maxOutputRange;
    }
}
