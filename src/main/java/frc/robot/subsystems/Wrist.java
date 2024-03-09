package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.configs.WristConfig;

public class Wrist extends SubsystemBase {

    public enum WristPosition {
        FLOOR(0.36),
        REST(0.0),
        AMP(0.27),
        TEST(0.3);

        public double position;

        private WristPosition(double position) {
            this.position = position;
        }
    }

    private AbsoluteEncoder mEncoder;
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private SparkPIDController mPID;
    private WristPosition currentPosition;
    private boolean atSetpoint;

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

        this.currentPosition = WristPosition.REST;
    }

    public void setDesiredPosition(WristPosition position) {
        this.currentPosition = position;
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public boolean atSetpoint() {
        atSetpoint = false;

        if (currentPosition.position - mEncoder.getPosition() < .05){
            atSetpoint = true;
        }    
        return atSetpoint;
    }


    @Override
    public void periodic() {
         mPID.setReference(currentPosition.position, ControlType.kPosition);
        //Shuffleboard.getTab("Wrist").addBoolean("At setpoint", Wrist.atSetpoint);
        Shuffleboard.getTab("Wrist").addDouble("Encoder Postion", mEncoder::getPosition);
    }
}
