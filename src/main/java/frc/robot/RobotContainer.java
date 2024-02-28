// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.ColorSensorV3;

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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SOTA_SwerveDrive;
import frc.robot.subsystems.SOTA_SwerveModule;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Wrist.WristPosition;
import frc.robot.subsystems.configs.ClimberConfig;
import frc.robot.subsystems.configs.DeliveryConfig;
import frc.robot.subsystems.configs.ArmConfig;
import frc.robot.subsystems.configs.IntakeConfig;
import frc.robot.subsystems.configs.SOTA_SwerveDriveConfig;
import frc.robot.subsystems.configs.SOTA_SwerveModuleConfig;
import frc.robot.subsystems.configs.ShooterConfig;
import frc.robot.subsystems.configs.WristConfig;

public class RobotContainer {
    public enum LimeLightPipelines {
        SPEAKER(0),
        AMP(1),
        STAGE(2);
        public int id;
        private LimeLightPipelines(int id) {
            this.id = id;
        }
    }
  private ConfigUtils mConfigUtils;

  private SendableChooser<Command> autoChooser;

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

  private Arm mArm;

  public RobotContainer() {
    this.mConfigUtils = new ConfigUtils();
    CameraServer.startAutomaticCapture();
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
      ArmConfig armConfig = mConfigUtils.readFromClassPath(ArmConfig.class, "arm/arm");
      SOTA_CompositeMotor leftMotor = lCompositeMotorFactory.generateCompositeMotor(armConfig.getLeftMotor());
      SOTA_CompositeMotor rightMotor = lCompositeMotorFactory.generateCompositeMotor(armConfig.getRightMotor());

      DoubleSolenoid leftSolenoid = new DoubleSolenoid(20, PneumaticsModuleType.REVPH, 2, 3);
      DoubleSolenoid rightSolenoid = new DoubleSolenoid(20, PneumaticsModuleType.REVPH, 0, 1);

      leftSolenoid.set(Value.kReverse);
      rightSolenoid.set(Value.kReverse);
      this.mArm = new Arm(armConfig, leftMotor.getMotor(), rightMotor.getMotor(), leftMotor.getAbsEncoder(),
          rightMotor.getAbsEncoder(), leftSolenoid,
          rightSolenoid);
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
      this.autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData(autoChooser);

    } catch (Exception e) {
      e.printStackTrace();
    }

    try {
      IntakeConfig intakeConfig = mConfigUtils.readFromClassPath(IntakeConfig.class, "intake/intake");
      SOTA_MotorController intakeMotor = MotorControllerFactory.generateMotorController(intakeConfig.getMotorConfig());
      DigitalInput proxSensor = new DigitalInput(2);
      this.mIntake = new Intake(intakeMotor, intakeConfig, proxSensor);
    } catch (Exception e) {
      e.printStackTrace();
    }
    configureDefaultCommands();
    configureBindings();
  }

  private void configureDefaultCommands() {
    mSwerveDrive.setDefaultCommand(
        new DriveCommand(mSwerveDrive, dController::getLeftY, dController::getLeftX, dController::getRightX));

    mArm.setDefaultCommand(Commands.run(() -> mArm.goToPosition(), mArm));

    rightClimber.setDefaultCommand(new Climb(leftClimber, rightClimber));
    mShooter.setDefaultCommand(Commands.runOnce(() -> mShooter.goToSpecifiedAngle(0 ), mShooter));
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

    mController.x().onTrue(new ShooterSequence(mShooter, mDelivery, mIntake, mWrist, mSwerveDrive))
        .onFalse(Commands.runOnce(() -> {
          mIntake.stop();
          mDelivery.stop();
          mShooter.stopFlyWheel();
        }, mIntake, mDelivery, mShooter));

    mController.y().onTrue(Commands.runOnce(() -> {
      mArm.setDesiredPosition(ArmPosition.AMP);
      mWrist.setDesiredPosition(WristPosition.AMP);
    }, mArm, mWrist));

    mController.povDown().onTrue(Commands.runOnce(() -> {
      mArm.setDesiredPosition(ArmPosition.REST);
      mWrist.setDesiredPosition(WristPosition.REST);
    }, mArm, mWrist));

    mController.start().onTrue(new Climb(leftClimber, rightClimber));
    mController.back().whileTrue(new ParallelCommandGroup(Commands.run(() -> leftClimber.stopMotor(), leftClimber),
        Commands.run(() -> rightClimber.stopMotor(), rightClimber)));

    mController.leftTrigger().onTrue(new Uppies(leftClimber, rightClimber))
        .onFalse(new ParallelCommandGroup(Commands.runOnce(() -> leftClimber.stopMotor(), leftClimber),
            Commands.runOnce(() -> rightClimber.stopMotor(), rightClimber)));

    mController.leftBumper().onTrue(Commands.run(() -> {
      mDelivery.toIntake();
      mIntake.intake();
    }, mDelivery)).onFalse(Commands.runOnce(() -> {
      mDelivery.stop();
      mIntake.stop();
    }, mDelivery, mIntake));

    mController.rightBumper().onTrue(Commands.run(() -> {
      mDelivery.toShooter();
      mIntake.outtake();
    }, mDelivery)).onFalse(Commands.runOnce(() -> {
      mDelivery.stop();
      mIntake.stop();
    }, mDelivery, mIntake));

    mController.rightTrigger().onTrue(new RotateToAprilTag(mSwerveDrive));

    mController.povUp().onTrue(Commands.runOnce(() -> {
      mArm.setDesiredPosition(ArmPosition.AMP);
      mWrist.setDesiredPosition(WristPosition.AMP);
    }, mArm, mWrist));

    mController.povLeft().onTrue(Commands.runOnce(() -> mArm.setDesiredPosition(ArmPosition.REST), mArm));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
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
