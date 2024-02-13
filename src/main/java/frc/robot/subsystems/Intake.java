package frc.robot.subsystems;

import java.util.Optional;

import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MultiplexedColorSensor;
import frc.robot.subsystems.configs.IntakeConfig;

public class Intake extends SubsystemBase {
    private SOTA_MotorController mMotor; 
    private MultiplexedColorSensor leftSensor;
    private MultiplexedColorSensor rightSensor;
    private double intakeVoltage;
    private int sensorTrigger;

    public Intake(SOTA_MotorController motor, IntakeConfig config, MultiplexedColorSensor leftSensor, MultiplexedColorSensor rightSensor) {
        this.mMotor = motor;
        this.leftSensor = leftSensor;
        this.rightSensor = rightSensor;
        Optional.ofNullable(config).ifPresent((IntakeConfig lConfig) -> this.intakeVoltage = lConfig.getIntakeVoltage());
        Optional.ofNullable(config).ifPresent((IntakeConfig lConfig) -> this.sensorTrigger = lConfig.getSensorTrigger());
        Shuffleboard.getTab("Intake").addBoolean("Has Note", this::hasNote);
        Shuffleboard.getTab("Intake").addNumber("Left Sensor Prox", leftSensor::getProximity);
        Shuffleboard.getTab("Intake").addNumber("Right Sensor Prox", rightSensor::getProximity);
    }

    public void intake() {
        mMotor.setVoltage(intakeVoltage);
    }

    public void outtake() {
        mMotor.setVoltage(-intakeVoltage);
    }

    public void stop() {
        mMotor.stopMotor();
    }

    public boolean hasNote() {
        boolean output;

        if (leftSensor.getProximity() > sensorTrigger || rightSensor.getProximity() > sensorTrigger) {
            output = true;
        } else {
            output = false;
        }

        return output;
    }
}
