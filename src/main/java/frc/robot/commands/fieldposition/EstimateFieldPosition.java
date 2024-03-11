// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.fieldposition;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.WheelPositions;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SOTA_SwerveDrive;

public class EstimateFieldPosition extends Command {
  /** Creates a new EstimateFieldPosition. */
  PoseEstimator m_PoseEstimator;
  SOTA_SwerveDrive m_Sota_SwerveDrive;
  Pose2d currentPose2d;


  public EstimateFieldPosition(SOTA_SwerveDrive in_Sota_SwerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Sota_SwerveDrive = in_Sota_SwerveDrive;
    currentPose2d = m_Sota_SwerveDrive.getPose();

    m_PoseEstimator = new PoseEstimator(
      m_Sota_SwerveDrive.getModuleKinematics(), 
      m_Sota_SwerveDrive.getModuleOdometry(), 
      VecBuilder.fill(.7, .7, 999999), 
      VecBuilder.fill(.7, .7, 999999));
  }


  public void updateOdometry() {
    m_PoseEstimator.updateWithTime(
        ,
        m_Sota_SwerveDrive.getGyro().getRotation2d(),
        m_Sota_SwerveDrive.getModulePositions());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateOdometry();
    
  }

}
