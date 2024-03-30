package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer.LimeLightPipelines;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.swerve.RotateToAprilTag;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SOTA_SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPosition;

public class ShooterSequence extends SequentialCommandGroup {
    private Shooter shooter;

    public ShooterSequence(Shooter mShooter, Delivery mDelivery, Intake mIntake, Wrist mWrist,
            SOTA_SwerveDrive mSwerve) {
        this.shooter = mShooter;
        addCommands(
                Commands.runOnce(() -> {
                    mWrist.setDesiredPosition(WristPosition.REST);
                    LimelightHelpers.setPipelineIndex("", LimeLightPipelines.SPEAKER.id);                    
                }, mWrist),
                Commands.waitUntil(mWrist::atSetpoint),
                // Commands.run(() -> {mIntake.outtake(); mDelivery.toIntake();}, mIntake,
                // mWrist).until(mIntake::hasNote),
                new RotateToAprilTag(mSwerve),
                Commands.parallel(
                        Commands.run(() -> {
                            mShooter.spinUpFlyWheel();
                            mShooter.goToAngle();
                        }, mShooter).until(this::isReadyToShoot),
                        Commands.waitUntil(this::isReadyToShoot).andThen(Commands.run(() -> {
                            mIntake.intake();
                            mDelivery.toShooter();
                        }, mIntake, mDelivery))),
                Commands.waitSeconds(0.5).andThen(Commands.runOnce(() -> {
                    mShooter.stopFlyWheel();
                    mIntake.stop();
                    mDelivery.stop();
                    LimelightHelpers.setPipelineIndex("", LimeLightPipelines.MEGATAG.id);
                }, mShooter)));

        addRequirements(mShooter, mDelivery, mIntake, mWrist);
    }

    public boolean isReadyToShoot() {
        return shooter.isAtShootingSpeed() && shooter.isAtAngle();
    }
}
