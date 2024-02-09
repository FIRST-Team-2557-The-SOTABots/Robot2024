package frc.robot.subsystems;

import SOTAlib.Encoder.Absolute.SOTA_AbsoulteEncoder;
import SOTAlib.Math.Conversions;
import SOTAlib.MotorController.NullConfigException;
import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.configs.SOTA_SwerveDriveConfig;
import frc.robot.subsystems.configs.SOTA_SwerveModuleConfig;

public class SOTA_SwerveModule {
    private String moduleName;
    private double kWheelCircumfrence;
    private double kGearRatio;

    private SOTA_MotorController angleMotor;
    private SOTA_AbsoulteEncoder angleEncoder;
    private PIDController anglePID;

    private SOTA_MotorController speedMotor;
    private PIDController speedPID;
    private SimpleMotorFeedforward speedFF;

    public SOTA_SwerveModule(SOTA_SwerveDriveConfig driveConfig, SOTA_SwerveModuleConfig moduleConfig,
            SOTA_MotorController angleMotor, SOTA_AbsoulteEncoder angleEncoder,
            SOTA_MotorController speedMotor) throws NullConfigException {
        if (driveConfig == null) {
            throw new NullConfigException("Swerve Module: No driveConfig");
        }

        if (moduleConfig == null) {
            throw new NullConfigException("Swerve Module: No moduleConfig");
        }

        this.moduleName = moduleConfig.getModuleName();
        this.kWheelCircumfrence = driveConfig.getWheelDiameter();
        this.kGearRatio = driveConfig.getGearRatio();

        this.angleMotor = angleMotor;
        this.angleEncoder = angleEncoder;
        this.anglePID = new PIDController(driveConfig.getAngleP(), driveConfig.getAngleI(), driveConfig.getAngleD());
        anglePID.enableContinuousInput(0, 1);

        this.speedMotor = speedMotor;
        this.speedPID = new PIDController(driveConfig.getSpeedP(), driveConfig.getSpeedI(), driveConfig.getSpeedD());
        this.speedFF = new SimpleMotorFeedforward(driveConfig.getSpeedKs(), driveConfig.getSpeedKv());
        Shuffleboard.getTab("Swerve").addNumber("Swerve Angle" + moduleName, this.angleEncoder::getConstrainedPositon);
    }

    public void setModule(SwerveModuleState swerveModuleState) {
        swerveModuleState = SwerveModuleState.optimize(swerveModuleState, getCurrentAngle());

        double anglePIDOutput = anglePID.calculate(getCurrentAngle().getRotations(),
                swerveModuleState.angle.getRotations());
        double speedRPM = Conversions.metersPerSecondToRPM(swerveModuleState.speedMetersPerSecond, kWheelCircumfrence,
                kGearRatio);
        double speedPIDOutput = speedPID.calculate(speedMotor.getEncoderVelocity(), speedRPM);
        double speedFFOutput = speedFF.calculate(speedRPM);

        if (swerveModuleState.speedMetersPerSecond == 0) {
            anglePIDOutput = 0;
            speedPIDOutput = 0;
            speedFFOutput = 0;
        }
        angleMotor.setVoltage(anglePIDOutput);
        speedMotor.setVoltage(speedFFOutput + speedPIDOutput);
    }

    public Rotation2d getCurrentAngle() {
        return new Rotation2d(Conversions.rotsToRads(angleEncoder.getConstrainedPositon()));
    }

}
