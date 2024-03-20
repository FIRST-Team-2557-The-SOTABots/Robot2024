package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SOTA_SwerveDrive;

public class RotateAndDrive extends Command {
    SOTA_SwerveDrive swerve;
    
    DoubleSupplier fwdSup;
    DoubleSupplier strSup;
    double angle;

    PIDController rotationPID;

    public RotateAndDrive (SOTA_SwerveDrive swerve, DoubleSupplier fwdSup, DoubleSupplier strSup, double angle) {
        this.swerve = swerve;
        
        this.fwdSup = fwdSup;
        this.strSup = strSup;
        this.angle = angle;

        this.rotationPID = new PIDController(0.12, 0, 0.01);
        rotationPID.setTolerance(0.5);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        rotationPID.setSetpoint(angle);
    }

    @Override
    public void execute() {
        double fwd = Math.signum(fwdSup.getAsDouble()) * fwdSup.getAsDouble() * fwdSup.getAsDouble();
        double str = Math.signum(strSup.getAsDouble()) * strSup.getAsDouble() * strSup.getAsDouble();
        double rot = rotationPID.calculate(adjustDegrees(swerve.getHeadingDegrees()));

        swerve.drive(new ChassisSpeeds(fwd, str, rot));
    }

    @Override
    public boolean isFinished() {
        return rotationPID.atSetpoint();
    }  

    public double adjustDegrees(double degrees) {
        return MathUtil.inputModulus(degrees, 0, 360);
    }
}
