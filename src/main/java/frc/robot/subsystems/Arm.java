package frc.robot.subsystems;

import SOTAlib.Encoder.Absolute.SOTA_AbsoulteEncoder;
import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.configs.ArmConfig;

public class Arm extends SubsystemBase {
    private SOTA_MotorController leftMotor;
    private SOTA_MotorController rightMotor;
    private SOTA_AbsoulteEncoder leftEncoder;
    private SOTA_AbsoulteEncoder rightEncoder;
    private DoubleSolenoid leftSolenoid;
    private DoubleSolenoid rightSolenoid;
    private ArmFeedforward mArmFeedforward;
    private PIDController mPidController;
    private ArmPosition currentPosition;

    public enum ArmPosition {
        REST(0),
        VERTICAL(0.25),
        AMP(0.27);

        public double position;

        private ArmPosition(double position) {
            this.position = position;
        }
    }

    public Arm(ArmConfig config, SOTA_MotorController leftMotor, SOTA_MotorController rightMotor,
            SOTA_AbsoulteEncoder leftEncoder, SOTA_AbsoulteEncoder rightEncoder, DoubleSolenoid leftSolenoid,
            DoubleSolenoid rightSolenoid) {

        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.leftSolenoid = leftSolenoid;
        this.rightSolenoid = rightSolenoid;
        this.mArmFeedforward = new ArmFeedforward(config.getkS(), config.getkG(), config.getkV());
        this.mPidController = new PIDController(config.getP(), config.getI(), config.getD());

        this.currentPosition = ArmPosition.REST;
        Shuffleboard.getTab("Arm").addDouble("Left Encoder Positon", this::getCorrectedLeftPosition);
        Shuffleboard.getTab("Arm").addDouble("Right Encoder Pos", this::getCorrectedRightPosition);
        Shuffleboard.getTab("Arm").addNumber("Setpoint", this::getCurrentSetpoint);
    }

    public double getCorrectedLeftPosition() {
        if (leftEncoder.getPosition() >= 0.95) {
            return 0.0;
        } else {
            return leftEncoder.getPosition();
        }
    }

    public double getCorrectedRightPosition() {
        if (rightEncoder.getPosition() >= 0.95) {
            return 0.0;
        } else {
            return rightEncoder.getPosition();
        }
    }

    public void goToPosition() {
        setMotorVoltage(mPidController.calculate(getCorrectedLeftPosition(), currentPosition.position));
    }

    private void setMotorVoltage(double volts) {
        if ((getCorrectedLeftPosition() <= 0.01 || getCorrectedRightPosition() <= 0.01) && volts < 0) {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        } else if ((getCorrectedLeftPosition() >= 0.3 && volts > 0)) {
            leftMotor.setVoltage(0);
            rightMotor.setVoltage(0);
        } else {
            leftMotor.setVoltage(volts);
            rightMotor.setVoltage(volts);
        }
    }

    public void setDesiredPosition(ArmPosition position) {
        this.currentPosition = position;
    }

    private double getCurrentSetpoint() {
        return currentPosition.position;
    }
}
