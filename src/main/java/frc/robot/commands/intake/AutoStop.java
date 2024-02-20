package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPosition;

public class AutoStop extends SequentialCommandGroup {

    public AutoStop(Wrist mWrist, Intake mIntake) {
        addCommands(Commands.runOnce(() -> mWrist.setDesiredPosition(WristPosition.FLOOR), mWrist),
                Commands.run(() -> mIntake.intake(), mIntake).until(mIntake::hasNote),
                Commands.runOnce(() -> {
                    mIntake.stop();
                    mWrist.setDesiredPosition(WristPosition.REST);
                }, mIntake, mWrist));
        addRequirements(mIntake, mWrist);
    }
}
