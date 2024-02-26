package frc.robot.commands.auto.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Spinup extends Command{
    
    Shooter mShooter;

    public Spinup (Shooter shooter) {
        this.mShooter = shooter;
    }

    @Override
    public void execute() {
        mShooter.spinUpFlyWheel();
    }

    @Override
    public boolean isFinished() {
        return mShooter.isAtShootingSpeed();
    }

}
