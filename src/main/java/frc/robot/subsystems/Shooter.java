package frc.robot.subsystems;

import SOTAlib.Encoder.Absolute.SOTA_AbsoulteEncoder;
import SOTAlib.Math.Conversions;
import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.configs.ShooterConfig;

public class Shooter extends SubsystemBase {
    private SOTA_MotorController linearActuator;
    private SOTA_AbsoulteEncoder linearEncoder;
    private PIDController linearPID;
    private double maxLinearValue;
    private final double angleConvM;
    private final double angleConvB;

    private double speakerTagHeight;
    private double speakerTagToHood;
    private double limeLightHeight;
    private double limeLightAngle;
    private double limeLightToShooterPivot;
    private double pivotHeight;

    private SOTA_MotorController leftShooter;
    private SOTA_MotorController rightShooter;
    private SimpleMotorFeedforward flyWheelFeedforward;
    private double targetVoltage;

    public Shooter(ShooterConfig config, SOTA_MotorController linearActuator, SOTA_AbsoulteEncoder linearEncoder,
            SOTA_MotorController leftShooter, SOTA_MotorController rightShooter) {

        this.linearActuator = linearActuator;
        this.linearEncoder = linearEncoder;

        this.linearPID = new PIDController(config.getP(), config.getI(), config.getD());
        this.maxLinearValue = config.getMaxLinearValue();
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
        this.flyWheelFeedforward = new SimpleMotorFeedforward(config.getkS(), config.getkV());
        this.targetVoltage = config.getTargetVoltage();

        Shuffleboard.getTab("Shooter").addDouble("Encoder Position", this.linearEncoder::getPosition);
        Shuffleboard.getTab("Shooter").addDouble("Encoder Raw Position", this.linearEncoder::getRawPosition);
        Shuffleboard.getTab("Shooter").addDouble("Angle to Hood", this::calcAngleToHood);
    }

    public void linearActuatorSetVoltage(double volts) {
        if ((linearEncoder.getPosition() >= maxLinearValue && linearEncoder.getPosition() <= 0.95)
                && Math.signum(volts) == 1) {
            linearActuator.stopMotor();
        } else {
            linearActuator.setVoltage(volts);
        }
    }

    public void spinUpFlyWheel() {
        double output = flyWheelFeedforward.calculate(targetVoltage);
        leftShooter.setVoltage(output);
        rightShooter.setVoltage(output);
    }

    public void stopFlyWheel() {
        leftShooter.stopMotor();
        rightShooter.stopMotor();
    }

    public boolean isAtShootingSpeed() {
        return leftShooter.getEncoderVelocity() >= targetVoltage && rightShooter.getEncoderVelocity() >= targetVoltage;
    }

    public boolean isNotAtShootingSpeed() {
        return !isAtShootingSpeed();
    }

    public double encoderToAngle(double encoderPos) {
        return angleConvM * encoderPos + angleConvB;
    }

    public double calcDistanceLimeLightToTag() {
        return (speakerTagHeight - limeLightHeight) / Math.tan(Math.toRadians(limeLightAngle
                + NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));
    }

    public double calcAngleToHood() {
        return Math.toDegrees(Math.atan((speakerTagToHood + speakerTagHeight - pivotHeight)/ (calcDistanceLimeLightToTag() + limeLightToShooterPivot)));
    }

    @Override
    public void periodic() {
        double volts = linearPID.calculate(encoderToAngle(linearEncoder.getPosition()), calcAngleToHood());
        linearActuatorSetVoltage(volts);
    }
}
