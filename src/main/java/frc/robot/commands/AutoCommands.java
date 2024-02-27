package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SOTA_SwerveDrive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPosition;

public class AutoCommands {
    
    private Shooter mShooter;
    private Intake mIntake;
    private Wrist mWrist;
    private Delivery mDelivery;
    private SOTA_SwerveDrive mSwerve;

    public AutoCommands (Shooter shooter, Intake intake, Wrist wrist, Delivery delivery, SOTA_SwerveDrive swerve) {
        this.mShooter = shooter;
        this.mIntake = intake;
        this.mWrist = wrist;
        this.mDelivery = delivery;
        this.mSwerve = swerve;

    }

    public Command intakeAutoStop () {
        return Commands.run(() -> {
            mWrist.setDesiredPosition(WristPosition.FLOOR);
            mIntake.intake();
        }, mIntake, mWrist).until(mIntake::hasNote).andThen(Commands.runOnce(() -> {
            mIntake.stop();
            mWrist.setDesiredPosition(WristPosition.REST);
        }, mIntake, mWrist));
    }

    public Command spinUpShoot () {
        return Commands.sequence(
            Commands.parallel(
                Commands.run(() -> {
                    mShooter.spinUpFlyWheel();
                    mShooter.goToAngle();
                }, mShooter).until(this::isReadyToShoot),
                Commands.waitUntil(this::isReadyToShoot).andThen(
                    Commands.runOnce(() -> {
                        mIntake.intake();
                        mDelivery.toShooter();
                    })
                )
            ),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> {
                mShooter.stopFlyWheel();
                mDelivery.stop();
                mIntake.stop();
            }, mShooter, mDelivery, mIntake)
        );
    }

    public boolean isReadyToShoot() {
        return mShooter.isAtShootingSpeed() && mShooter.isAtAngle();
    }

}
