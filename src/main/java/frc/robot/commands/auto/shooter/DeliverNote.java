package frc.robot.commands.auto.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;

public class DeliverNote extends Command {
    
    Delivery mDelivery;
    Intake mIntake;

    public DeliverNote (Delivery delivery, Intake intake) {
        this.mDelivery = delivery;
        this.mIntake = intake;
    }

    @Override
    public void execute() {
        mDelivery.toShooter();
        mIntake.intake();
    }
}
