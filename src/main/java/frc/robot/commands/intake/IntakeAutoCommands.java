package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPosition;

public class IntakeAutoCommands {
    
    private Intake mIntake;
    private Wrist mWrist;

    public IntakeAutoCommands (Intake intake, Wrist wrist) {
        this.mIntake = intake;
        this.mWrist = wrist;
    }

    public Command intakeAutoStop () {
        return Commands.run(() -> {
            mWrist.setDesiredPosition(WristPosition.FLOOR);
            mIntake.intake();
        }, mIntake, mWrist).until(mIntake::hasNote).andThen(Commands.run(() -> {
            mIntake.stop();
            mWrist.setDesiredPosition(WristPosition.REST);
        }, mIntake, mWrist));
    }

}
