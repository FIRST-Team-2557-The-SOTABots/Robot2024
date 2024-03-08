package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SOTA_SwerveDrive;

public class RotateToAngle extends Command {
    private SOTA_SwerveDrive mDrive;
    private PIDController mPidController;
    private DoubleSupplier targetAngle;

    public RotateToAngle(SOTA_SwerveDrive drive, DoubleSupplier targetAngle) {
        this.mDrive = drive;
        this.targetAngle = targetAngle;
        this.mPidController = new PIDController(0.12, 0.0, 0.01);
        mPidController.setTolerance(0.5);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("RotateToAngle", targetAngle.getAsDouble());
        mPidController.setSetpoint(targetAngle.getAsDouble());
    }

    @Override
    public void execute() {
        mDrive.drive(new ChassisSpeeds(0, 0, mPidController.calculate(wrapHeading(mDrive.getHeadingDegrees()))));
    }

    public double wrapHeading(double angle) {
        return MathUtil.inputModulus(angle, 0, 360);
    }

    @Override
    public boolean isFinished() {
        return mPidController.atSetpoint();
    }
}
