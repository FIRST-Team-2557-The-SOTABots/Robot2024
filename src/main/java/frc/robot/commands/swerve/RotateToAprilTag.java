package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SOTA_SwerveDrive;

public class RotateToAprilTag extends Command {
    private SOTA_SwerveDrive mDrive;
    private PIDController mRotationController;
    private double xOffset;

    public RotateToAprilTag (SOTA_SwerveDrive drive) {
        this.mDrive = drive;
        mRotationController = new PIDController(0, 0, 0);
    }

    public double calculateRotationSpeed () {
        xOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        return mRotationController.calculate(xOffset, 0);
    }

    public boolean isAtAprilTag () {
        return Math.abs(xOffset) < 2;

    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, calculateRotationSpeed());
        mDrive.drive(speeds);
    }
    @Override
    public boolean isFinished() {
        return Math.abs(xOffset) < 2;

    }


}
