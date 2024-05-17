// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

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
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.climber.Climb;
import frc.robot.commands.climber.Uppies;
import frc.robot.commands.intake.AutoStop;
import frc.robot.commands.shooter.ShooterSequence;
import frc.robot.commands.swerve.DriveCommand;
import frc.robot.commands.swerve.RotateAndDrive;
import frc.robot.commands.swerve.RotateToAprilTag;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SOTA_SwerveDrive;
import frc.robot.subsystems.SOTA_SwerveModule;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPosition;
import frc.robot.subsystems.configs.ArmConfig;
import frc.robot.subsystems.configs.ClimberConfig;
import frc.robot.subsystems.configs.DeliveryConfig;
import frc.robot.subsystems.configs.IntakeConfig;
import frc.robot.subsystems.configs.SOTA_SwerveDriveConfig;
import frc.robot.subsystems.configs.SOTA_SwerveModuleConfig;
import frc.robot.subsystems.configs.ShooterConfig;
import frc.robot.subsystems.configs.WristConfig;

public class RobotContainer {
    public enum LimeLightPipelines {
        SPEAKER(4),
        AMP(1),
        STAGE(2),
        MEGATAG(0);

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

    public RobotContainer(DoubleSupplier getPeriod) {
        this.mConfigUtils = new ConfigUtils();
        CameraServer.startAutomaticCapture();
        this.dController = new SOTA_Xboxcontroller(0);
        this.mController = new SOTA_Xboxcontroller(1);
                this.mGyro = new NavX(new AHRS(Port.kMXP));

        try {
            DeliveryConfig deliveryConfig = mConfigUtils.readFromClassPath(DeliveryConfig.class, "delivery/delivery");
            SOTA_MotorController deliveryMotor = MotorControllerFactory
                    .generateMotorController(deliveryConfig.getDeliveryConfig());
            SOTA_MotorController shooterDeliveryMotor = MotorControllerFactory
                    .generateMotorController(deliveryConfig.getShooterDeliveryConfig());
            DigitalInput shooterNoteSensor = new DigitalInput(3);
            this.mDelivery = new Delivery(deliveryConfig, deliveryMotor, shooterDeliveryMotor, shooterNoteSensor);
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

            ArmConfig armConfig = mConfigUtils.readFromClassPath(ArmConfig.class, "arm/arm");
            CANSparkMax armLeftMotor = new CANSparkMax(armConfig.getLeftMotorPort(), MotorType.kBrushless);
            CANSparkMax armRightMotor = new CANSparkMax(armConfig.getRightMotorPort(), MotorType.kBrushless);
            AbsoluteEncoder armLeftEncoder = armLeftMotor
                    .getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle);
            SparkPIDController armPID = armLeftMotor.getPIDController();

            mArm = new Arm(armConfig, armPID, armLeftEncoder, armLeftMotor, armRightMotor);

        } catch (Exception e) {
            e.printStackTrace();
        }
        try {
            WristConfig wristConfig = mConfigUtils.readFromClassPath(WristConfig.class, "wrist/wrist");
            CANSparkMax wristLeftMotor = new CANSparkMax(wristConfig.getLeftMotorPort(), MotorType.kBrushless);
            CANSparkMax wristRightMotor = new CANSparkMax(wristConfig.getRightMotorPort(), MotorType.kBrushless);
            AbsoluteEncoder wristEncoder = wristLeftMotor
                    .getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle);
            SparkPIDController wristPID = wristLeftMotor.getPIDController();

            mWrist = new Wrist(wristConfig, wristPID, wristEncoder, wristLeftMotor, wristRightMotor);

        } catch (Exception e) {
            e.printStackTrace();
        }

        try {
            ClimberConfig leftConfig = mConfigUtils.readFromClassPath(ClimberConfig.class, "climber/left");
            ClimberConfig rightConfig = mConfigUtils.readFromClassPath(ClimberConfig.class, "climber/right");

            SOTA_MotorController leftMotor = MotorControllerFactory
                    .generateMotorController(leftConfig.getMotorConfig());
            SOTA_MotorController rightMotor = MotorControllerFactory
                    .generateMotorController(rightConfig.getMotorConfig());

            DigitalInput leftSwitch = new DigitalInput(leftConfig.getSwitchPort());
            DigitalInput rightSwitch = new DigitalInput(rightConfig.getSwitchPort());

            this.leftClimber = new Climber(leftConfig, leftMotor, leftSwitch);
            this.rightClimber = new Climber(rightConfig, rightMotor, rightSwitch);
        } catch (Exception e) {
            e.printStackTrace();
        }

        try {
            CompositeMotorFactory mCompositeMotorFactory = new CompositeMotorFactory();

            SOTA_SwerveDriveConfig driveConfig = mConfigUtils.readFromClassPath(SOTA_SwerveDriveConfig.class,
                    "swerve/drive");

            SwerveDriveKinematics kinematics = new SwerveDriveKinematics(driveConfig.generateModuleTranslations());

            SOTA_SwerveModule[] modules = {
                    initModule(mConfigUtils, mCompositeMotorFactory, "swerve/frontright", driveConfig),
                    initModule(mConfigUtils, mCompositeMotorFactory, "swerve/frontleft", driveConfig),
                    initModule(mConfigUtils, mCompositeMotorFactory, "swerve/backleft", driveConfig),
                    initModule(mConfigUtils, mCompositeMotorFactory, "swerve/backright", driveConfig)
            };

            this.mSwerveDrive = new SOTA_SwerveDrive(modules, kinematics, mGyro, driveConfig, getPeriod);

        } catch (Exception e) {
            e.printStackTrace();
        }

        try {
            IntakeConfig intakeConfig = mConfigUtils.readFromClassPath(IntakeConfig.class, "intake/intake");
            SOTA_MotorController intakeMotor = MotorControllerFactory
                    .generateMotorController(intakeConfig.getMotorConfig());
            intakeMotor.setInverted(true);
            DigitalInput proxSensor = new DigitalInput(2);
            this.mIntake = new Intake(intakeMotor, intakeConfig, proxSensor);
        } catch (Exception e) {
            e.printStackTrace();
        }

        registerNamedCommands();

        this.autoChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Competition").add("Auto Chooser", autoChooser);

        configureDefaultCommands();
        configureBindings();
    }

    private void registerNamedCommands() {
        AutoCommands autoCommands = new AutoCommands(mShooter, mIntake, mWrist, mDelivery, mArm, mSwerveDrive);
        NamedCommands.registerCommand("Shoot", autoCommands.spinUpShoot(false, false));
        NamedCommands.registerCommand("Shoot preloaded", autoCommands.spinUpShoot(true, true));
        NamedCommands.registerCommand("Rotate Shoot", autoCommands.spinUpShoot(false, true));
        NamedCommands.registerCommand("Intake", autoCommands.intakeAutoStop());
        NamedCommands.registerCommand("Run Intake", autoCommands.intakeAmp());
        NamedCommands.registerCommand("Spin Flywheels", autoCommands.setFlyWheels());
        NamedCommands.registerCommand("Spin Flywheels to RPM", autoCommands.runFlywheelsToRpm());
        NamedCommands.registerCommand("Stop Flywheels", autoCommands.stopFlyWheels());
        NamedCommands.registerCommand("Align Shoot", autoCommands.alignAndShoot());
        // NamedCommands.registerCommand("Arm to Amp", autoCommands.setArmToAmp());
        // NamedCommands.registerCommand("Arm to Rest", autoCommands.setArmToRest());
        NamedCommands.registerCommand("Align Tag", new RotateToAprilTag(mSwerveDrive));
        NamedCommands.registerCommand("Amp Sequence", autoCommands.scoreInAmp());
        NamedCommands.registerCommand("Align Shooter", autoCommands.alignShooter());
        NamedCommands.registerCommand("Reset Gyro", Commands.runOnce(() -> mSwerveDrive.resetHeading(), mSwerveDrive));
        NamedCommands.registerCommand("Teleop shoot command",
                new ShooterSequence(mShooter, mDelivery, mIntake, mWrist, mSwerveDrive));
        NamedCommands.registerCommand("Check Intake", autoCommands.checkIntake());
        NamedCommands.registerCommand("Outtake", Commands.run(() -> mIntake.outtake(), mIntake).withTimeout(0.75)
                .andThen(Commands.runOnce(() -> mIntake.stop(), mIntake)));
        NamedCommands.registerCommand("Intake Ground",
                Commands.runOnce(() -> mWrist.setDesiredPosition(WristPosition.FLOOR), mWrist));
        NamedCommands.registerCommand("Limelight Speaker",
                Commands.runOnce(() -> LimelightHelpers.setPipelineIndex("", LimeLightPipelines.SPEAKER.id)));
        NamedCommands.registerCommand("Limelight Odom",
                Commands.runOnce(() -> LimelightHelpers.setPipelineIndex("", LimeLightPipelines.MEGATAG.id)));
        NamedCommands.registerCommand("Spit note", autoCommands.spitNote());
        NamedCommands.registerCommand("Reset Intake", Commands.runOnce(() -> {
            mWrist.setDesiredPosition(WristPosition.REST);
            mIntake.stop();
        }));
        NamedCommands.registerCommand("Feed Note", autoCommands.feedNote());
    }

    private void configureDefaultCommands() {

        mShooter.setDefaultCommand(Commands.run(() -> mShooter.linearActuatorSetVoltage(mController.getLeftY()*-12), mShooter));
  
    }

    private void configureBindings() {

        mController.a().onTrue(Commands.run(() -> {
            mIntake.intake();
            mDelivery.toShooter();
        }, mIntake, mDelivery).until(mDelivery::shooterHasNote).andThen(
            Commands.runOnce(() -> {
                mIntake.stop();
                mDelivery.stop();
            }, mIntake, mDelivery)
        )).onFalse(Commands.runOnce(() -> {
            mIntake.stop();
            mDelivery.stop();
        }, mIntake, mDelivery));

        mController.x().onTrue(Commands.run(() -> {
            mShooter.spinToSetRPM();
        }, mShooter)).onFalse(Commands.runOnce(() -> {
            mShooter.stopFlyWheel();
        }, mShooter));

        mController.rightBumper().onTrue(Commands.runOnce(() -> mDelivery.toShooter(), mDelivery)).onFalse(
            Commands.runOnce(() -> mDelivery.stop(), mDelivery));
    }

    public Command getAutonomousCommand() {
        mGyro.resetAngle();
        LimelightHelpers.setPipelineIndex("", LimeLightPipelines.MEGATAG.id);
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
