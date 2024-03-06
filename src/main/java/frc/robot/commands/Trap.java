package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer.LimeLightPipelines;
import frc.robot.commands.swerve.RotateToAngle;
import frc.robot.subsystems.SOTA_SwerveDrive;

public class Trap extends SequentialCommandGroup {

    public enum TrapGyroAngles {
        STAGE_LEFT(0),
        STAGE_RIGHT(0),
        BACK_STAGE(180);

        public double angle;

        TrapGyroAngles(double angle){
            this.angle = angle;
        }
    } 

    public Trap(SOTA_SwerveDrive drive) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(LimeLightPipelines.STAGE.id);
        addCommands(new RotateToAngle(drive, TrapGyroAngles.BACK_STAGE.angle));
        addRequirements(drive);
    }
}
