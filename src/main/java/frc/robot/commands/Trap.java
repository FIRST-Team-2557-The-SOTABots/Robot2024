package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer.LimeLightPipelines;
import frc.robot.commands.swerve.RotateToAngle;
import frc.robot.subsystems.SOTA_SwerveDrive;

public class Trap extends SequentialCommandGroup {

    public enum TrapGyroAngles {
        STAGE_LEFT(120),
        STAGE_RIGHT(240),
        CENTER_STAGE(0),
        ERROR(0);

        public double angle;

        TrapGyroAngles(double angle) {
            this.angle = angle;
        }
    }

    public Trap(SOTA_SwerveDrive drive) {

        addCommands(new RotateToAngle(drive, this::calcTargetAngle));

        addRequirements(drive);
    }

    public double calcTargetAngle() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline")
                .setNumber(LimeLightPipelines.STAGE.id);
        long tagID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid")
                .getInteger(-1);
        TrapGyroAngles targetAngle;
        if (tagID == 12 || tagID == 16) {
            targetAngle = TrapGyroAngles.STAGE_RIGHT;
            SmartDashboard.putBoolean("Worked", true);
        } else if (tagID == 11 || tagID == 15) {
            targetAngle = TrapGyroAngles.STAGE_LEFT;
        } else if (tagID == 13 || tagID == 14) {
            targetAngle = TrapGyroAngles.CENTER_STAGE;
        } else {
            SmartDashboard.putBoolean("Worked", false);
            targetAngle = TrapGyroAngles.ERROR;
        }
        return targetAngle.angle;
    }

}
