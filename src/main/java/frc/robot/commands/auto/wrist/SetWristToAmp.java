package frc.robot.commands.auto.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPosition;

public class SetWristToAmp extends Command {
    private Wrist mWrist;

    public SetWristToAmp (Wrist wrist) {
        this.mWrist = wrist;
    }

    @Override
    public void execute() {
        mWrist.setDesiredPosition(WristPosition.AMP);
    }

    @Override
    public boolean isFinished() {
        return mWrist.atSetpoint();
    }
}
