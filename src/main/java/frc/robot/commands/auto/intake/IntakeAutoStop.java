package frc.robot.commands.auto.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeAutoStop extends Command{
    
    Intake mIntake;

    public IntakeAutoStop (Intake intake) {
        this.mIntake = intake;
    }

    @Override
    public void execute() {
        mIntake.intake();
    }

    @Override
    public boolean isFinished() {
        return mIntake.hasNote();
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.stop();
    }

}
