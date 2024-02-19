package frc.robot.subsystems;

import SOTAlib.Encoder.Absolute.SOTA_AbsoulteEncoder;
import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.configs.ShooterConfig;

public class Shooter extends SubsystemBase {
    private SOTA_MotorController linearActuator;
    private SOTA_AbsoulteEncoder linearEncoder;
    private PIDController linearPID;
    private double maxLinearValue;

    private SOTA_MotorController leftShooter;
    private SOTA_MotorController rightShooter;

    public Shooter(ShooterConfig config, SOTA_MotorController linearActuator, SOTA_AbsoulteEncoder linearEncoder,
            SOTA_MotorController leftShooter, SOTA_MotorController rightShooter) {

        this.linearActuator = linearActuator;
        this.linearEncoder = linearEncoder;

        this.linearPID = new PIDController(config.getP(), config.getI(), config.getD());
        this.maxLinearValue = config.getMaxLinearValue();

        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;
        Shuffleboard.getTab("Shooter").addDouble("Encoder Position", this.linearEncoder::getPosition);
        Shuffleboard.getTab("Shooter").addDouble("Encoder Raw Position", this.linearEncoder::getRawPosition);
    }

    public void linearActuatorSetVoltage(double volts) {
        if ((linearEncoder.getPosition() >= maxLinearValue && linearEncoder.getPosition() <= 0.95) && Math.signum(volts) == 1) {
            linearActuator.stopMotor();
        } else {
            linearActuator.setVoltage(volts);
        }
    }

    public void runShooters(double speed) {
        leftShooter.set(speed);
        rightShooter.set(speed);
    }

    @Override
    public void periodic() {
        // linearPID.calculate(linearEncoder.getPosition(), );
    }
}
