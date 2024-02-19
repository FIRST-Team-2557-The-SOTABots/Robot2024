package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Climber;

public class Uppies extends ParallelCommandGroup {
    public Uppies(Climber leftClimber, Climber rightClimber) {
        addCommands(Commands.run(() -> leftClimber.downClimb(), leftClimber),
                Commands.run(() -> rightClimber.downClimb(), rightClimber));
        addRequirements(leftClimber, rightClimber);
    }
}
