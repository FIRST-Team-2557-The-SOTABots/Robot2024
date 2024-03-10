package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import SOTAlib.Encoder.Absolute.SOTA_AbsoulteEncoder;
import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.configs.ArmConfig;

public class Arm extends SubsystemBase {
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private AbsoluteEncoder mEncoder;
    private SparkPIDController mPID;
    private ArmPosition currentPosition;

    public enum ArmPosition {
        REST(0.1),
        VERTICAL(0.25),
        AMP(0.2);

        public double position;

        private ArmPosition(double position) {
            this.position = position;
        }
    }

    public Arm(ArmConfig config, SparkPIDController armPID, AbsoluteEncoder armLeftEncoder, CANSparkMax armLeftMotor,
            CANSparkMax armRightMotor) {

        this.leftMotor = armLeftMotor;
        this.rightMotor = armLeftMotor;
        this.mEncoder = armLeftEncoder;
        this.mPID = armPID;

        leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightMotor.setInverted(config.getRightMotorInverted());
        leftMotor.setInverted(config.getLeftMotorInverted());
        rightMotor.follow(leftMotor, true);

        mEncoder.setInverted(true);

        mPID.setP(config.getP());
        mPID.setI(config.getI());
        mPID.setD(config.getD());
        mPID.setFeedbackDevice(mEncoder);
        mPID.setPositionPIDWrappingEnabled(true);
        mPID.setOutputRange(config.getMinOutputRange(), config.getMaxOutputRange());
        //hi - lauren
        this.currentPosition = ArmPosition.REST;

        Shuffleboard.getTab("Arm").addDouble("Left Encoder Positon", this::getCorrectedLeftPosition);
        Shuffleboard.getTab("Arm").addNumber("Setpoint", this::getCurrentSetpoint);
        Shuffleboard.getTab("Arm").addBoolean("At setpoint", this::isAtSetpoint);
    }

    public double getCorrectedLeftPosition() {
        if (mEncoder.getPosition() >= 0.9) {
            return 0.0;
        } else {
            return mEncoder.getPosition();
        }
    }


    // private void setMotorVoltage(double volts) {
    //     // if ((getCorrectedLeftPosition() <= 0.01 || getCorrectedRightPosition() <= 0.01) && volts < 0) {
    //     //     leftMotor.stopMotor();
    //     //     rightMotor.stopMotor();
    //     //     SmartDashboard.putBoolean("Stopped 1", true);
    //     // } else if ((getCorrectedLeftPosition() >= 0.32 && volts > 0)) {
    //     //     leftMotor.setVoltage(0);
    //     //     rightMotor.setVoltage(0);
    //     //     SmartDashboard.putBoolean("Stopped 2", true);
    //     // } else {
    //     //     leftMotor.setVoltage(volts);
    //     //     rightMotor.setVoltage(volts);
    //     //     SmartDashboard.putBoolean("Stopped 1", false);
    //     //     SmartDashboard.putBoolean("Stopped 2", false);
    //     // }
    //     // SmartDashboard.putNumber("volts", volts);
    //     leftMotor.setVoltage(volts);
    //     rightMotor.setVoltage(volts);
    // }

    public boolean isAtSetpoint(){
        if (getCorrectedLeftPosition() - currentPosition.position < .05){
            return true;
        } else{
            return false;
        }
    }

    public void setDesiredPosition(ArmPosition position) {
        this.currentPosition = position;
    }

    private double getCurrentSetpoint() {
        return currentPosition.position;
    }

    @Override
    public void periodic() {
         mPID.setReference(currentPosition.position, ControlType.kPosition);
         SmartDashboard.putNumber("encoder position", getCorrectedLeftPosition());
        //Shuffleboard.getTab("Wrist").addBoolean("At setpoint", Wrist.atSetpoint);
        //Shuffleboard.getTab("Wrist").addDouble("Current Encoder Postion", mEncoder::getPosition);
    }
}
