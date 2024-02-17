package frc.robot.subsystems;

import SOTAlib.Encoder.Absolute.SOTA_AbsoulteEncoder;
import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.configs.ArmConfig;

public class Arm extends SubsystemBase {
    private SOTA_MotorController leftMotor;
    private SOTA_MotorController rightMotor;
    private SOTA_AbsoulteEncoder mEncoder;
    private ArmFeedforward mArmFeedforward;
    private PIDController mPidController;

    public Arm(ArmConfig config, SOTA_MotorController leftMotor, SOTA_MotorController rightMotor, SOTA_AbsoulteEncoder encoder) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.mEncoder = encoder;
        this.mArmFeedforward = new ArmFeedforward(config.getkS(), config.getkG(), config.getkV());
        this.mPidController = new PIDController(config.getP(), config.getI(), config.getD());
        Shuffleboard.getTab("Arm").addDouble("Encoder Positon", mEncoder::getPosition);
    }

    public void moveArms(double speed) {
        leftMotor.set(MathUtil.clamp(speed, -0.3, 0.3));
        rightMotor.set(MathUtil.clamp(speed, -0.3, 0.3));
    }

    public void setVoltage(double volts) {
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
    }

}
