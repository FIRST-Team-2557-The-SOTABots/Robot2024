package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.RotateToAprilTag;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPosition;

public class Shoot extends Command{
    private boolean hasShot = false;
    private Shooter mShooter;
    private Intake mIntake;
    private Delivery mDelivery;
    private Wrist mWrist;

    public Shoot (Shooter mShooter, Intake mIntake, Delivery mDelivery, Wrist mWrist) {
        this.mShooter = mShooter;
        this.mIntake = mIntake;
        this.mDelivery = mDelivery;
        this.mWrist = mWrist;
    }

    @Override
    public void execute() {
        new SequentialCommandGroup(Commands.runOnce(() -> {mWrist.setDesiredPosition(WristPosition.REST);}, mWrist),
            Commands.waitUntil(mWrist::atSetpoint),
            // Commands.run(() -> {mIntake.outtake(); mDelivery.toIntake();}, mIntake, mWrist).until(mIntake::hasNote),
            // new RotateToAprilTag(mSwerve),
            Commands.parallel(
                Commands.run(() -> {
                    mShooter.spinUpFlyWheel();
                    mShooter.goToAngle();
                }, mShooter),
                Commands.waitUntil(this::isReadyToShoot).andThen(Commands.run(() -> {mIntake.intake(); mDelivery.toShooter();}, mIntake, mDelivery))),
                Commands.waitSeconds(0.5).andThen(Commands.runOnce(() -> {mShooter.stopFlyWheel(); hasShot = true;})));
    }

    @Override
    public boolean isFinished() {
        return hasShot;
    }

    @Override
    public void end(boolean interrupted) {
        mShooter.stopFlyWheel();
    }

    public boolean isReadyToShoot() {
        return mShooter.isAtShootingSpeed() && mShooter.isAtAngle();
    }
}
