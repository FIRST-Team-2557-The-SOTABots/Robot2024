package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer.LimeLightPipelines;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.swerve.RotateToAprilTag;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SOTA_SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPosition;

public class AutoShoot extends Command {
    private Shooter mShooter;
    private Delivery mDelivery;
    private Intake mIntake;
    private Wrist mWrist;
    private SOTA_SwerveDrive mSwerve;

    boolean hasShot;

    public AutoShoot(Shooter mShooter, Delivery mDelivery, Intake mIntake, Wrist mWrist, SOTA_SwerveDrive mSwerve) {
        this.mShooter = mShooter;
        this.mDelivery = mDelivery;
        this.mIntake = mIntake;
        this.mWrist = mWrist;
        this.mSwerve = mSwerve;
    }

    @Override
    public void initialize() {
        new SequentialCommandGroup(Commands.runOnce(() -> {mWrist.setDesiredPosition(WristPosition.REST);}, mWrist),
            Commands.waitUntil(mWrist::atSetpoint),
            // Commands.run(() -> {mIntake.outtake(); mDelivery.toIntake();}, mIntake, mWrist).until(mIntake::hasNote),
            new RotateToAprilTag(mSwerve),
            Commands.parallel(
                Commands.run(() -> {
                    mShooter.spinUpFlyWheel();
                    mShooter.goToAngle();
                }, mShooter),
                Commands.waitUntil(this::isReadyToShoot).andThen(Commands.run(() -> {mIntake.intake(); mDelivery.toShooter();}, mIntake, mDelivery))
            ),
            Commands.waitSeconds(0.5).andThen(Commands.runOnce(() -> {mShooter.stopFlyWheel(); mDelivery.stop(); mIntake.stop();}, mShooter, mDelivery, mIntake))
        );
    }

    @Override
    public boolean isFinished() {
        return hasShot;        
    }
    
    @Override
    public void end(boolean interrupted) {
        mShooter.stopFlyWheel();
        mDelivery.stop();
        mIntake.stop();
    }

    public boolean isReadyToShoot() {
        return mShooter.isAtShootingSpeed() && mShooter.isAtAngle();
    }
}
