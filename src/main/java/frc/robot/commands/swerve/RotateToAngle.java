package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SOTA_SwerveDrive;

public class RotateToAngle extends Command {
    private SOTA_SwerveDrive mDrive;
    private PIDController mPidController;

    public RotateToAngle(SOTA_SwerveDrive drive, double targetAngle) {
        this.mDrive = drive;
        this.mPidController = new PIDController(0.12, 0.0, 0.01);
        mPidController.setTolerance(0.5);
        mPidController.setSetpoint(targetAngle);
    }

    @Override
    public void execute() {
        mDrive.drive(new ChassisSpeeds(0, 0, mPidController.calculate(mDrive.getHeading().getDegrees())));
    }

    @Override
    public boolean isFinished() {
        return mPidController.atSetpoint();
    }
}
