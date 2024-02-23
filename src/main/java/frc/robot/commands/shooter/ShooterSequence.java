package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.swerve.RotateToAprilTag;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SOTA_SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPosition;

public class ShooterSequence extends SequentialCommandGroup {

    public ShooterSequence(Shooter mShooter, Delivery mDelivery, Intake mIntake, Wrist mWrist, SOTA_SwerveDrive mSwerve) {
       addCommands(
            Commands.runOnce(() -> {mWrist.setDesiredPosition(WristPosition.REST);}, mWrist),
            Commands.waitUntil(mWrist::atSetpoint),
            // Commands.run(() -> {mIntake.outtake(); mDelivery.toIntake();}, mIntake, mWrist).until(mIntake::hasNote),
            new RotateToAprilTag(mSwerve),
            Commands.parallel(
                Commands.run(() -> {mShooter.spinUpFlyWheel();}, mShooter),
                Commands.waitUntil(mShooter::isAtShootingSpeed).andThen(Commands.run(() -> {mIntake.intake(); mDelivery.toShooter();}, mIntake, mDelivery))
            )
        );

        addRequirements(mShooter, mDelivery, mIntake, mWrist);
    }
}
