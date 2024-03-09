package frc.robot.subsystems.configs;

public class WristConfig {
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

    

    public boolean getLeftMotorInverted() {
        return leftMotorInverted;
    }


    public double getMinOutputRange() {
        return minOutputRange;
    }


    public int getLeftMotorPort() {
        return leftMotorPort;
    }

    public int getRightMotorPort() {
        return rightMotorPort;
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


    public double getMaxOutputRange() {
        return maxOutputRange;
    }


    public boolean getRightMotorInverted() {
        return rightMotorInverted;
    }
}
