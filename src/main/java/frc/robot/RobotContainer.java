// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import SOTAlib.Config.ConfigUtils;
import SOTAlib.Control.SOTA_Xboxcontroller;
import SOTAlib.Encoder.Absolute.SOTA_AbsoulteEncoder;
import SOTAlib.Factories.CompositeMotorFactory;
import SOTAlib.Factories.EncoderFactory;
import SOTAlib.Factories.IllegalMotorModel;
import SOTAlib.Factories.MotorControllerFactory;
import SOTAlib.Gyro.NavX;
import SOTAlib.Gyro.SOTA_Gyro;
import SOTAlib.MotorController.SOTA_CompositeMotor;
import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.climber.Climb;
import frc.robot.commands.climber.Uppies;
import frc.robot.commands.intake.AutoStop;
import frc.robot.commands.shooter.ShooterSequence;
import frc.robot.commands.swerve.DriveCommand;
import frc.robot.commands.swerve.RotateToAprilTag;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SOTA_SwerveDrive;
import frc.robot.subsystems.SOTA_SwerveModule;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPosition;
import frc.robot.subsystems.configs.ClimberConfig;
import frc.robot.subsystems.configs.DeliveryConfig;
import frc.robot.subsystems.configs.IntakeConfig;
import frc.robot.subsystems.configs.SOTA_SwerveDriveConfig;
import frc.robot.subsystems.configs.SOTA_SwerveModuleConfig;
import frc.robot.subsystems.configs.ShooterConfig;
import frc.robot.subsystems.configs.WristConfig;

public class RobotContainer {
  private ConfigUtils mConfigUtils;

  private SOTA_Xboxcontroller dController;
  private SOTA_Xboxcontroller mController;
  private SOTA_Gyro mGyro;

  private SOTA_SwerveDrive mSwerveDrive;

  private Intake mIntake;
  private Wrist mWrist;
  private Climber leftClimber;
  private Climber rightClimber;
  private Delivery mDelivery;
  private Shooter mShooter;

  public RobotContainer() {
    this.mConfigUtils = new ConfigUtils();

    this.dController = new SOTA_Xboxcontroller(0);
    this.mController = new SOTA_Xboxcontroller(1);
    this.mGyro = new NavX(new AHRS(Port.kMXP));

    try {
      DeliveryConfig deliveryConfig = mConfigUtils.readFromClassPath(DeliveryConfig.class, "delivery/delivery");
      SOTA_MotorController deliveryMotor = MotorControllerFactory
          .generateMotorController(deliveryConfig.getDeliveryConfig());
      this.mDelivery = new Delivery(deliveryConfig, deliveryMotor);
    } catch (Exception e) {
      e.printStackTrace();
    }

    try {
      ShooterConfig shooterConfig = mConfigUtils.readFromClassPath(ShooterConfig.class, "shooter/shooter");
      SOTA_MotorController linearActuator = MotorControllerFactory
          .generateMotorController(shooterConfig.getLinearActuatorConfig());
      SOTA_AbsoulteEncoder lineaEncoder = EncoderFactory
          .generateAbsoluteEncoder(shooterConfig.getLinearEncoderConfig());
      SOTA_MotorController leftMotor = MotorControllerFactory
          .generateMotorController(shooterConfig.getLeftShooterConfig());
      SOTA_MotorController rightMotor = MotorControllerFactory
          .generateMotorController(shooterConfig.getRightShooterConfig());

      this.mShooter = new Shooter(shooterConfig, linearActuator, lineaEncoder, leftMotor, rightMotor);
    } catch (Exception e) {
      e.printStackTrace();
    }

    try {
      CompositeMotorFactory lCompositeMotorFactory = new CompositeMotorFactory();
      WristConfig wristConfig = mConfigUtils.readFromClassPath(WristConfig.class, "wrist/wrist");
      SOTA_CompositeMotor leftMotor = lCompositeMotorFactory.generateCompositeMotor(wristConfig.getLeftMotor());
      SOTA_MotorController rightMotor = MotorControllerFactory.generateMotorController(wristConfig.getRightMotor());
      this.mWrist = new Wrist(wristConfig, leftMotor.getAbsEncoder(), leftMotor.getMotor(), rightMotor);
    } catch (Exception e) {
      e.printStackTrace();
    }

    try {
      ClimberConfig leftConfig = mConfigUtils.readFromClassPath(ClimberConfig.class, "climber/left");
      ClimberConfig rightConfig = mConfigUtils.readFromClassPath(ClimberConfig.class, "climber/right");

      SOTA_MotorController leftMotor = MotorControllerFactory.generateMotorController(leftConfig.getMotorConfig());
      SOTA_MotorController rightMotor = MotorControllerFactory.generateMotorController(rightConfig.getMotorConfig());

      DigitalInput leftSwitch = new DigitalInput(leftConfig.getSwitchPort());
      DigitalInput rightSwitch = new DigitalInput(rightConfig.getSwitchPort());

      this.leftClimber = new Climber(leftConfig, leftMotor, leftSwitch);
      this.rightClimber = new Climber(rightConfig, rightMotor, rightSwitch);
    } catch (Exception e) {
      e.printStackTrace();
    }

    try {
      CompositeMotorFactory mCompositeMotorFactory = new CompositeMotorFactory();

      SOTA_SwerveDriveConfig driveConfig = mConfigUtils.readFromClassPath(SOTA_SwerveDriveConfig.class, "swerve/drive");

      SwerveDriveKinematics kinematics = new SwerveDriveKinematics(driveConfig.generateModuleTranslations());

      SOTA_SwerveModule[] modules = {
          initModule(mConfigUtils, mCompositeMotorFactory, "swerve/frontright", driveConfig),
          initModule(mConfigUtils, mCompositeMotorFactory, "swerve/frontleft", driveConfig),
          initModule(mConfigUtils, mCompositeMotorFactory, "swerve/backleft", driveConfig),
          initModule(mConfigUtils, mCompositeMotorFactory, "swerve/backright", driveConfig)
      };

      this.mSwerveDrive = new SOTA_SwerveDrive(modules, kinematics, mGyro, driveConfig);
      configureDefaultCommands();

    } catch (Exception e) {
      e.printStackTrace();
    }

    try {
      IntakeConfig intakeConfig = mConfigUtils.readFromClassPath(IntakeConfig.class, "intake/intake");
      SOTA_MotorController intakeMotor = MotorControllerFactory.generateMotorController(intakeConfig.getMotorConfig());
      MultiplexedColorSensor leftSensor = new MultiplexedColorSensor(Port.kMXP, 0);
      MultiplexedColorSensor rightSensor = new MultiplexedColorSensor(Port.kMXP, 1);
      this.mIntake = new Intake(intakeMotor, intakeConfig, leftSensor, rightSensor);
    } catch (Exception e) {
      e.printStackTrace();
    }
    configureBindings();
  }

  private void configureDefaultCommands() {
    mSwerveDrive.setDefaultCommand(
        new DriveCommand(mSwerveDrive, dController::getLeftY, dController::getLeftX, dController::getRightX));
  }

  private void configureBindings() {
    dController.leftBumper().onTrue(Commands.runOnce(() -> mSwerveDrive.setFieldCentric(false), mSwerveDrive));
    dController.rightBumper().onTrue(Commands.runOnce(() -> mSwerveDrive.setFieldCentric(true), mSwerveDrive));
    dController.start().onTrue(Commands.runOnce(() -> mSwerveDrive.resetHeading(), mSwerveDrive));

    mController.a().onTrue(new AutoStop(mWrist, mIntake)).onFalse(Commands.runOnce(() -> {
      mWrist.setDesiredPosition(WristPosition.REST);
      mIntake.stop();
    }, mWrist, mIntake));

    mController.b().onTrue(Commands.run(() -> mIntake.outtake(), mIntake))
        .onFalse(Commands.runOnce(() -> mIntake.stop(), mIntake));

    mController.x().onTrue(new ShooterSequence(mShooter, mDelivery, mIntake, mWrist, mSwerveDrive)).onFalse(Commands.runOnce(() -> {
      mIntake.stop();
      mDelivery.stop();
      mShooter.stopFlyWheel();
    }, mIntake, mDelivery, mShooter));

    mController.start().onTrue(new Climb(leftClimber, rightClimber));
    mController.back().onTrue(new ParallelCommandGroup(Commands.runOnce(() -> leftClimber.stopMotor(), leftClimber),
        Commands.runOnce(() -> rightClimber.stopMotor(), rightClimber)));

    mController.leftTrigger().onTrue(new Uppies(leftClimber, rightClimber))
        .onFalse(new ParallelCommandGroup(Commands.runOnce(() -> leftClimber.stopMotor(), leftClimber),
            Commands.runOnce(() -> rightClimber.stopMotor(), rightClimber)));

    mController.leftBumper().onTrue(Commands.run(() -> {
      mDelivery.toIntake();
    }, mDelivery)).onFalse(Commands.runOnce(() -> {
      mDelivery.stop();
    }, mDelivery));

    mController.rightBumper().onTrue(Commands.run(() -> {
      mDelivery.toShooter();
    }, mDelivery)).onFalse(Commands.runOnce(() -> {
      mDelivery.stop();
    }, mDelivery));
    
    mController.rightTrigger().onTrue(new RotateToAprilTag(mSwerveDrive));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private SOTA_SwerveModule initModule(ConfigUtils lConfigUtils, CompositeMotorFactory lCompositeMotorFactory,
      String swerveModuleConfigPath, SOTA_SwerveDriveConfig driveConfig) throws IllegalMotorModel, Exception {

    SOTA_SwerveModuleConfig moduleConfig = lConfigUtils.readFromClassPath(SOTA_SwerveModuleConfig.class,
        swerveModuleConfigPath);
    SOTA_CompositeMotor angleSystem = lCompositeMotorFactory.generateCompositeMotor(moduleConfig.getAngleSystem());
    SOTA_MotorController speedMotor = MotorControllerFactory.generateMotorController(moduleConfig.getSpeedConfig());
    return new SOTA_SwerveModule(driveConfig, moduleConfig, angleSystem.getMotor(), angleSystem.getAbsEncoder(),
        speedMotor);

  }
}
