package frc.robot.subsystems;


import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.configs.IntakeConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class Intake extends SubsystemBase {
    private SOTA_MotorController mMotor;
    private DigitalInput proxSensor;
    private double intakeVoltage;

    public Intake(SOTA_MotorController motor, IntakeConfig config, DigitalInput proxSwitch) {
        this.mMotor = motor;
        this.proxSensor = proxSwitch;
        Shuffleboard.getTab("Intake").addBoolean("Has Note", this::hasNote);
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
        return proxSensor.get();
    }
}
