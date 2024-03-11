package frc.robot.subsystems;

import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.configs.DeliveryConfig;

public class Delivery extends SubsystemBase {
    private SOTA_MotorController mDeliveryMotor;
    private SOTA_MotorController mShooterDeliveryMotor;
    private double intakeSpeed;

    public Delivery(DeliveryConfig config, SOTA_MotorController deliveryMotor, SOTA_MotorController shooterDeliveryMotor) {
        this.mDeliveryMotor = deliveryMotor;
        this.mShooterDeliveryMotor = shooterDeliveryMotor;
        this.intakeSpeed = config.getDeliverySpeed();
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
