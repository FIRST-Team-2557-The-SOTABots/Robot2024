package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPosition;

public class ShooterSequence extends SequentialCommandGroup {

    public ShooterSequence(Shooter mShooter, Delivery mDelivery, Intake mIntake, Wrist mWrist) {
       addCommands(
            Commands.runOnce(() -> {mWrist.setDesiredPosition(WristPosition.REST);}, mWrist),
            Commands.waitUntil(mWrist::atSetpoint),
            // Commands.run(() -> {mIntake.outtake(); mDelivery.toIntake();}, mIntake, mWrist).until(mIntake::hasNote),
            Commands.run(() -> {mShooter.spinUpFlyWheel();}, mShooter).until(mShooter::isAtShootingSpeed),
            Commands.parallel(
                Commands.run(() -> {mShooter.spinUpFlyWheel();}, mShooter),
                Commands.run(() -> {mIntake.intake(); mDelivery.toShooter();}, mIntake, mDelivery)
            ).until(mShooter::isNotAtShootingSpeed)
        );

        addRequirements(mShooter, mDelivery, mIntake, mWrist);
    }
}
