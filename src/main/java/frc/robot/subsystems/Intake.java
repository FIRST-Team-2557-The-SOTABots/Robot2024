package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.ColorSensorV3;

import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.configs.IntakeConfig;

public class Intake extends SubsystemBase {
    private SOTA_MotorController mMotor;
    private ColorSensorV3 colorSensor;
    private double intakeVoltage;
    private int sensorTrigger;

    public Intake(SOTA_MotorController motor, IntakeConfig config, ColorSensorV3 colorSensor) {
        this.mMotor = motor;
        this.colorSensor = colorSensor;
        Optional.ofNullable(config)
                .ifPresent((IntakeConfig lConfig) -> this.intakeVoltage = lConfig.getIntakeVoltage());
        Optional.ofNullable(config)
                .ifPresent((IntakeConfig lConfig) -> this.sensorTrigger = lConfig.getSensorTrigger());
        Shuffleboard.getTab("Intake").addBoolean("Has Note", this::hasNote);
        Shuffleboard.getTab("Intake").addNumber("Sensor Prox", colorSensor::getProximity);
    }

    public void intake() {
        mMotor.set(intakeVoltage);
    }

    public void outtake() {
        mMotor.set(-intakeVoltage);
    }

    public void stop() {
        mMotor.stopMotor();
    }

    public boolean hasNote() {
        boolean output;

        if (colorSensor.getProximity() >= sensorTrigger) {
            output = true;
        } else {
            output = false;
        }

        return output;
    }
}
