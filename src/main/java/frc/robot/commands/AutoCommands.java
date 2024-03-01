package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.swerve.RotateToAprilTag;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SOTA_SwerveDrive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Wrist.WristPosition;

public class AutoCommands {
    
    private Shooter mShooter;
    private Intake mIntake;
    private Wrist mWrist;
    private Delivery mDelivery;
    private Arm mArm;
    private SOTA_SwerveDrive mSwerve;

    public AutoCommands (Shooter shooter, Intake intake, Wrist wrist, Delivery delivery, Arm arm, SOTA_SwerveDrive swerve) {
        this.mShooter = shooter;
        this.mIntake = intake;
        this.mWrist = wrist;
        this.mDelivery = delivery;
        this.mArm = arm;
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
                // new RotateToAprilTag(mSwerve),
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

    public Command setFlyWheels () {
        return Commands.runOnce(() -> {
            mShooter.setSpeed(1);

        });
    }

    public Command stopFlyWheels () {
        return Commands.runOnce(() -> {
            mShooter.stopFlyWheel();
        });
    }

    public Command alignAndShoot () {
        return Commands.sequence(
            Commands.parallel(
                // new RotateToAprilTag(mSwerve),
                Commands.run(() -> {
                    mShooter.goToAngle();
                }, mShooter).until(this::isReadyToShoot),
                Commands.waitUntil(this::isReadyToShoot).andThen(
                    Commands.runOnce(() -> {
                        mIntake.intake();
                        mDelivery.toShooter();
                    })  
                )
            ),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> {
                mIntake.stop();
                mDelivery.stop();
            })
        );
    }

    // public Command alignAndShoot () {
    //     return Commands.sequence(
    //         // new RotateToAprilTag(mSwerve),
    //         Commands.run(() -> mShooter.goToAngle(), mShooter),
    //         // Commands.waitUntil(this::isReadyToShoot),
    //         Commands.waitSeconds(1),
    //         Commands.runOnce(() -> {
    //             mIntake.intake();
    //             mDelivery.toShooter();
    //         }, mIntake, mDelivery),
    //         Commands.waitSeconds(0.5),
    //         Commands.runOnce(() -> {
    //             mIntake.intake();
    //             mDelivery.toShooter();
    //         }, mIntake, mDelivery)
    //     );
    // }

    public Command setArmToAmp () {
        return Commands.runOnce(() -> {
            mArm.setDesiredPosition(ArmPosition.AMP);
            mWrist.setDesiredPosition(WristPosition.AMP);
        }).andThen(Commands.waitUntil(mArm::isAtSetpoint));
    }

    public Command setArmToRest () {
        return Commands.runOnce(() -> {
            mArm.setDesiredPosition(ArmPosition.REST);
            mWrist.setDesiredPosition(WristPosition.REST);
        }).andThen(Commands.waitUntil(mArm::isAtSetpoint));
    }

    
    public Command intakeAmp() {
        return Commands.sequence(
            Commands.runOnce(() -> mIntake.intake()),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> mIntake.stop())
        );
    }

    public boolean isReadyToShoot() {
        return mShooter.isAtShootingSpeed() && mShooter.isAtAngle() && mWrist.atSetpoint();
    }


}
