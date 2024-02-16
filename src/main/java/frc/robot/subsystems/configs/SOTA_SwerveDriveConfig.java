package frc.robot.subsystems.configs;

import SOTAlib.Math.Conversions;
import edu.wpi.first.math.geometry.Translation2d;

public class SOTA_SwerveDriveConfig {
    /**
     * Wheel Base (wheel distance front to back) in inches
     */
    private double wheelBase;

    /**
     * Track Width (wheel distance side to side) in inches
     */
    private double trackWidth;

    /**
     * max speed of the modules in feet per second
     */
    private double maxSpeed;

    /**
     * max rotational speed of the robot in radians per second
     */
    private double maxRotationalVelocity;

    /**
     * P constant for swerve module angle pid controller
     */
    private double angleP;

    /**
     * I constant for swerve module angle pid controller
     */
    private double angleI;

    /**
     * D constant for swerve module angle pid controller
     */
    private double angleD;

    /**
     * P constant for swerve module speed pid controller
     */
    private double speedP;

    /**
     * I constant for swerve module speed pid controller
     */
    private double speedI;

    /**
     * D constant for swerve module speed pid controller
     */
    private double speedD;

    /**
     * Kv constant for swerve speed feedforward
     */
    private double speedKv;

    /**
     * Ks constant for swerve speed feedforward
     */
    private double speedKs;

    /**
     * diameter of the swerve wheels in inches
     */
    private double wheelDiameter;

    /**
     * Ratio between the speed motor and the wheel output
     */
    private double gearRatio;

    /**
     * Order Front right, Front left, Back Left, Back Right
     */
    public Translation2d[] generateModuleTranslations() {
        Translation2d[] moduleTranslations = {
                new Translation2d(getWheelBaseMeters() / 2, -getTrackWidthMeters() / 2),
                new Translation2d(getWheelBaseMeters() / 2, getTrackWidthMeters() / 2),
                new Translation2d(-getWheelBaseMeters() / 2, getTrackWidthMeters() / 2),
                new Translation2d(-getWheelBaseMeters() / 2, -getTrackWidthMeters() / 2)
        };
        return moduleTranslations;
    }

    public double getWheelBaseMeters() {
        return Conversions.inchesToMeters(wheelBase);
    }

    public double getTrackWidthMeters() {
        return Conversions.inchesToMeters(trackWidth);
    }

    /**
     * @return math rotational velocity of the robot in radians per second
     */
    public double getMaxRotationalVelocity() {
        return maxRotationalVelocity;
    }

    /**
     * @return wheel base in inches
     */
    public double getWheelBase() {
        return wheelBase;
    }

    /**
     * @return track width in inches
     */
    public double getTrackWidth() {
        return trackWidth;
    }

    /**
     * @return max speed of the modules in feet per second
     */
    public double getMaxSpeed() {
        return maxSpeed;
    }

    public double getAngleP() {
        return this.angleP;
    }

    public double getAngleI() {
        return this.angleI;
    }

    public double getAngleD() {
        return this.angleD;
    }

    public double getSpeedP() {
        return this.speedP;
    }

    public double getSpeedI() {
        return this.speedI;
    }

    public double getSpeedD() {
        return this.speedD;
    }

    public double getSpeedKv() {
        return speedKv;
    }

    public double getSpeedKs() {
        return speedKs;
    }

    public double getWheelDiameter() {
        return wheelDiameter;
    }

    public double getGearRatio() {
        return this.gearRatio;
    }

}
