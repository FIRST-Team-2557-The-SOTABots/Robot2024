package frc.robot.subsystems.configs;

import SOTAlib.Config.MotorControllerConfig;

public class DeliveryConfig {
    private double deliverySpeed;

    private MotorControllerConfig deliveryConfig;
    private MotorControllerConfig shooterDeliveryConfig;

    public MotorControllerConfig getShooterDeliveryConfig() {
		return shooterDeliveryConfig;
	}

	public double getDeliverySpeed() {
        return deliverySpeed;
    }

    public MotorControllerConfig getDeliveryConfig() {
        return this.deliveryConfig;
    }
}
