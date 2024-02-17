package frc.robot.subsystems;

import SOTAlib.Encoder.Absolute.SOTA_AbsoulteEncoder;
import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.configs.WristConfig;

public class Wrist extends SubsystemBase {
    private SOTA_AbsoulteEncoder mEncoder;
    private SOTA_MotorController leftMotor;
    private SOTA_MotorController rightMotor;

    public Wrist(WristConfig config, SOTA_AbsoulteEncoder encoder, SOTA_MotorController leftMotor,
            SOTA_MotorController rightMotor) {
        this.mEncoder = encoder;
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        Shuffleboard.getTab("Wrist").addDouble("Encoder Postion", mEncoder::getPosition);
    }

    public void toFloor() {
        leftMotor.set(0.1);
        rightMotor.set(0.1);
    }

    public void toRest() {
        leftMotor.set(-0.1);
        rightMotor.set(-0.1);
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }
}
