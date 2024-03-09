package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import SOTAlib.Encoder.Absolute.SOTA_AbsoulteEncoder;
import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.configs.WristConfig;

public class Wrist extends SubsystemBase {

    // public enum WristPosition {
    //     FLOOR(0.36),
    //     REST(0.0),
    //     AMP(0.27),
    //     TEST(0.3);

    //     public double position;

    //     private WristPosition(double position) {
    //         this.position = position;
    //     }
    // }

    private AbsoluteEncoder mEncoder;
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private SparkPIDController mPID;
    private double setPoint;
    // private WristPosition currentPosition;

    public Wrist(WristConfig config, SparkPIDController wristPID, AbsoluteEncoder wristEncoder, CANSparkMax wristLeftMotor,
            CANSparkMax rightMotor) {
        this.mEncoder = wristEncoder;
        this.leftMotor = wristLeftMotor;
        this.rightMotor = rightMotor;
        this.mPID = wristPID;

        mPID.setP(config.getP());
        mPID.setI(config.getI());
        mPID.setD(config.getD());
        mPID.setFeedbackDevice(wristEncoder);
        mPID.setOutputRange(config.getMinOutputRange(), config.getMaxOutputRange());

        // this.currentPosition = WristPosition.REST;
        // this.mPID = new PIDController(config.getP(), config.getI(), config.getD());
        // mPID.enableContinuousInput(0, 1);
        // mPID.setTolerance(0.02);
        // Shuffleboard.getTab("Wrist").addBoolean("At setpoint", mPID::atSetpoint);
        // Shuffleboard.getTab("Wrist").addDouble("Encoder Postion", mEncoder::getPosition);
        // Shuffleboard.getTab("Wrist").addDouble("Adjusted Position", this::getCorrectedEncoderPosition);
    }

    public void setDesiredPosition(double setPoint){
        this.setPoint = setPoint;
    }

    // public void setDesiredPosition(WristPosition position) {
    //     this.currentPosition = position;
    // }

    /**
     * When the wrist goes to the rest sometimes the abs encoder underflows from 0
     * to .99999 ish.
     * First we tried doing a continous input pid with safety, but then the pid
     * wanted to go backwards.
     * So now this just corrects for the underflow, and we should be golden.
     * 
     * @return value of the encoder with the physical zero position taken into
     *         account
     */
    // public double getCorrectedEncoderPosition() {
    //     double output;
    //     if (mEncoder.getPosition() > 0.95) {
    //         output = mEncoder.getPosition() - 1;
    //     } else {
    //         output = mEncoder.getPosition();
    //     }
    //     return output;
    // }

    // public void toFloor() {
    //     leftMotor.set(0.1);
    //     rightMotor.set(0.1);
    // }

    // public void toRest() {
    //     leftMotor.set(-0.1);
    //     rightMotor.set(-0.1);
    // }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    // private void setWristVoltage(double volts) {
    //     if (mEncoder.getPosition() <= 0.95 && mEncoder.getPosition() >= 0.45) {
    //         stop();
    //     } else {
    //         leftMotor.setVoltage(volts);
    //         rightMotor.setVoltage(volts);
    //     }
    // }

    public boolean atSetpoint() {
        if (setPoint - mEncoder.getPosition() < .05){
            return true;
        }
        
        return false;
    }

    @Override
    public void periodic() {
         mPID.setReference(setPoint, ControlType.kPosition);
        // if (mPID.atSetpoint()) {
        //     mPID.reset();
        // }
        // setWristVoltage(mPID.calculate(getCorrectedEncoderPosition(), currentPosition.position));
    }
}
