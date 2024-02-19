package frc.robot.subsystems.configs;

import SOTAlib.Config.MotorControllerConfig;

public class DeliveryConfig {
    private double deliverySpeed;

    private MotorControllerConfig deliveryConfig;

    public double getDeliverySpeed() {
        return deliverySpeed;
    }

    public MotorControllerConfig getDeliveryConfig() {
        return this.deliveryConfig;
    }
}
