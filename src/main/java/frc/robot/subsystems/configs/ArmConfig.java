package frc.robot.subsystems.configs;


public class ArmConfig {

    private double p;
    private double i;
    private double d;
    private double iZone;
    private double minOutputRange;
    private double maxOutputRange;
    private int rightMotorPort;
    private boolean rightMotorInverted;
    private int leftMotorPort;
    private boolean leftMotorInverted;


    public boolean getLeftMotorInverted() {
        return leftMotorInverted;
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

    public double getiZone() {
        return iZone;
    }

    public double getMinOutputRange() {
        return minOutputRange;
    }

    public double getMaxOutputRange() {
        return maxOutputRange;
    }

    public int getRightMotorPort() {
        return rightMotorPort;
    }

    public boolean getRightMotorInverted() {
        return rightMotorInverted;
    }

    public int getLeftMotorPort() {
        return leftMotorPort;
    }
}
