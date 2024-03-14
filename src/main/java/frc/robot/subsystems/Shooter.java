package frc.robot.subsystems;

import SOTAlib.Encoder.Absolute.SOTA_AbsoulteEncoder;
import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.configs.ShooterConfig;

public class Shooter extends SubsystemBase {
    private SOTA_MotorController linearActuator;
    private SOTA_AbsoulteEncoder linearEncoder;
    private PIDController linearPID;
    private double maxLinearValue;
    private double minLinearValue;
    private double restLinearValue;
    private final double angleConvM;
    private final double angleConvB;
    private double angleCalcA;
    private double angleCalcB;
    private double angleCalcC;

    private double rpmCalcA;
    private double rpmCalcB;
    private double rpmCalcC;

    private double speakerTagHeight;
    private double speakerTagToHood;
    private double limeLightHeight;
    private double limeLightAngle;
    private double limeLightToShooterPivot;
    private double pivotHeight;

    private SOTA_MotorController leftShooter;
    private SOTA_MotorController rightShooter;
    private SimpleMotorFeedforward leftFF;
    private SimpleMotorFeedforward rightFF;
    private double targetRPM;
    private double kMaxShooterAngle;

    public Shooter(ShooterConfig config, SOTA_MotorController linearActuator, SOTA_AbsoulteEncoder linearEncoder,
            SOTA_MotorController leftShooter, SOTA_MotorController rightShooter) {

        this.linearActuator = linearActuator;
        this.linearEncoder = linearEncoder;

        this.linearPID = new PIDController(config.getP(), config.getI(), config.getD());
        this.linearPID.setTolerance(0.5);
        this.maxLinearValue = config.getMaxLinearValue();
        this.minLinearValue = config.getMinLinearValue();
        this.restLinearValue = config.getRestLinearValue();
        this.angleConvM = config.getAngleConvM();
        this.angleConvB = config.getAngleConvB();

        this.speakerTagHeight = config.getSpeakerTagHeight();
        this.speakerTagToHood = config.getSpeakerTagToHood();
        this.limeLightHeight = config.getLimeLightHeight();
        this.limeLightAngle = config.getLimeLightAngle();
        this.limeLightToShooterPivot = config.getLimeLightToShooterPivot();
        this.pivotHeight = config.getPivotHeight();

        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;
        this.leftFF = new SimpleMotorFeedforward(config.getLeftKS(), config.getLeftKV());
        this.rightFF = new SimpleMotorFeedforward(config.getRightKS(), config.getRightKV());
        this.targetRPM = config.getTargetRPM();
        this.kMaxShooterAngle = encoderToAngle(maxLinearValue);

        this.angleCalcA = config.getAngleCalcA();
        this.angleCalcB = config.getAngleCalcB();
        this.angleCalcC = config.getAngleCalcC();

        this.rpmCalcA = config.getRpmCalcA();
        this.rpmCalcB = config.getRpmCalcB();
        this.rpmCalcC = config.getRpmCalcC();

        Shuffleboard.getTab("Shooter").addDouble("Encoder Position", this.linearEncoder::getPosition);
        Shuffleboard.getTab("Shooter").addDouble("Encoder Raw Position", this.linearEncoder::getRawPosition);
        Shuffleboard.getTab("Shooter").addDouble("Angle Target", this::calcTargetAngle);
        Shuffleboard.getTab("Shooter").addDouble("RPM Target", this::calcTargetRpm);
        Shuffleboard.getTab("Shooter").addDouble("Shooter Angle", this::getShooterAngle);
        Shuffleboard.getTab("Shooter").addDouble("Left rpm", leftShooter::getEncoderVelocity);
        Shuffleboard.getTab("Shooter").addDouble("Right rpm", rightShooter::getEncoderVelocity);
        // Shuffleboard.getTab("Competition").addDouble("Left rpm", leftShooter::getEncoderVelocity);
        // Shuffleboard.getTab("Competition").addDouble("Right rpm", rightShooter::getEncoderVelocity);
        Shuffleboard.getTab("Competition").addBoolean("Too Far!", this::isTooFar);
        Shuffleboard.getTab("Competition").addBoolean("Ready To Shoot", this::isReadyToShoot);
        Shuffleboard.getTab("Shooter").addDouble("Corrected Position", this::getCorrectedEncoderPosition);
        Shuffleboard.getTab("Shooter").addBoolean("isAtShootingSpeed", this::isAtShootingSpeed);
        Shuffleboard.getTab("Shooter").addDouble("Distance to Limelight", this::calcDistanceLimeLightToTag);
        Shuffleboard.getTab("Shooter").addDouble("Linear Actuator Voltage", linearActuator::getMotorCurrent);
    }

    public double getCorrectedEncoderPosition() {
        double output;
        if (linearEncoder.getPosition() > 0.95) {
            output = 0.0;
        } else {
            output = linearEncoder.getPosition();
        }

        return output;
    }

    public double calcTargetAngle() {
        double x = calcDistanceLimeLightToTag();
        return (angleCalcA * x * x) + (angleCalcB * x) + angleCalcC;
    }

    public double calcTargetRpm() {
        double x = calcDistanceLimeLightToTag();
        return (rpmCalcA * x * x) + (rpmCalcB * x) + rpmCalcC;
    }

    public void linearActuatorSetVoltage(double volts) {
        if (getCorrectedEncoderPosition() > maxLinearValue && volts > 0) {
            linearActuator.stopMotor();
        } else if (getCorrectedEncoderPosition() < minLinearValue && volts < 0) {
            linearActuator.stopMotor();
        } else if (linearPID.atSetpoint()) {
            linearActuator.stopMotor();
        } else {
            linearActuator.setVoltage(volts);
        }
    }

    public void spinUpFlyWheel() {
        targetRPM = calcTargetRpm();
        leftShooter.setVoltage(leftFF.calculate(targetRPM));
        rightShooter.setVoltage(rightFF.calculate(targetRPM));
    }

    public void stopFlyWheel() {
        leftShooter.stopMotor();
        rightShooter.stopMotor();
    }

    public boolean isAtShootingSpeed() {
        return leftShooter.getEncoderVelocity() >= targetRPM - 100
                || rightShooter.getEncoderVelocity() >= targetRPM;
    }

    public boolean isNotAtShootingSpeed() {
        return !isAtShootingSpeed();
    }

    public double encoderToAngle(double encoderPos) {
        return angleConvM * encoderPos + angleConvB;
    }

    public double getShooterAngle() {
        return encoderToAngle(getCorrectedEncoderPosition());
    }

    public double calcDistanceLimeLightToTag() {
        return (speakerTagHeight - limeLightHeight) / Math.tan(Math.toRadians(limeLightAngle
                + NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));
    }

    public double calcAngleToHood() {
        return Math.toDegrees(Math.atan((speakerTagToHood + speakerTagHeight - pivotHeight)
                / (calcDistanceLimeLightToTag() + limeLightToShooterPivot)));
    }

    public void goToAngle() {
        double volts = linearPID.calculate(encoderToAngle(getCorrectedEncoderPosition()),
                calcTargetAngle());
        linearActuatorSetVoltage(volts);
    }

    public void goToSpecifiedAngle(double angle) {
        double volts = linearPID.calculate(encoderToAngle(getCorrectedEncoderPosition()), angle);
        linearActuatorSetVoltage(volts);
    }

    public void goToRestAngle() {
        double volts = linearPID.calculate(encoderToAngle(getCorrectedEncoderPosition()),
                encoderToAngle(restLinearValue));
        linearActuatorSetVoltage(volts);
    }

    public boolean isAtAngle() {
        return linearPID.atSetpoint();
    }

    public boolean isReadyToShoot() {
        return isAtAngle() && isAtShootingSpeed();
    }

    private boolean isTooClose() {
        return calcAngleToHood() > kMaxShooterAngle;
    }

    private boolean isTooFar() {
        return calcTargetAngle() < 24; // TODO: find actual angle using encoder units
    }

    public void setSpeed(double speed) {
        leftShooter.set(speed);
        rightShooter.set(speed);
    }

}
