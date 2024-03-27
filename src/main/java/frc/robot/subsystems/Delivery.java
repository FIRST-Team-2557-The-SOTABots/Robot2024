package frc.robot.subsystems;

import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.configs.DeliveryConfig;

public class Delivery extends SubsystemBase {
    private SOTA_MotorController mDeliveryMotor;
    private SOTA_MotorController mShooterDeliveryMotor;
    private DigitalInput mShooterNoteSensor;
    private double intakeSpeed;

    public Delivery(DeliveryConfig config, SOTA_MotorController deliveryMotor, SOTA_MotorController shooterDeliveryMotor, DigitalInput shooterNoteSensor) {
        this.mDeliveryMotor = deliveryMotor;
        this.mShooterDeliveryMotor = shooterDeliveryMotor;
        this.intakeSpeed = config.getDeliverySpeed();
        this.mShooterNoteSensor = shooterNoteSensor;
        Shuffleboard.getTab("Delivery").addDouble("Shooter Feeder Current",mShooterDeliveryMotor::getMotorCurrent);
        Shuffleboard.getTab("Delivery").addDouble("Delivery Current", mDeliveryMotor::getMotorCurrent);
        Shuffleboard.getTab("Delivery").addDouble("Delivery Encoder Pos", mDeliveryMotor::getEncoderPosition);
        Shuffleboard.getTab("Delivery").addBoolean("Shooter Has Note", this::shooterHasNote);
    }

    public boolean shooterHasNote() {
        return !mShooterNoteSensor.get();
    }

    public void toShooter() {
        mDeliveryMotor.set(intakeSpeed);
        mShooterDeliveryMotor.set(intakeSpeed);
    }

    public void toIntake() {
        mDeliveryMotor.set(-intakeSpeed);
        mShooterDeliveryMotor.set(-intakeSpeed);
    }

    public void stop() {
        mDeliveryMotor.stopMotor();
        mShooterDeliveryMotor.stopMotor();
    }
}
