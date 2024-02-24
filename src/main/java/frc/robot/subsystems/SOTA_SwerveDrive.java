package frc.robot.subsystems;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import SOTAlib.Gyro.SOTA_Gyro;
import SOTAlib.Math.Conversions;
import SOTAlib.MotorController.NullConfigException;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.configs.SOTA_SwerveDriveConfig;

public class SOTA_SwerveDrive extends SubsystemBase {
    private SwerveDriveKinematics mDriveKinematics;
    private SwerveDriveOdometry mDriveOdometry;
    private SOTA_SwerveModule[] modules;
    private SOTA_Gyro mGyro;
    private boolean fieldCentric;
    private Pose2d currentPose;
    private Field2d mField2d;

    private double MAX_SPEED;

    private ShuffleboardTab sTab;
    private double MAX_ROTATIONAL_VELOCITY;

    public SOTA_SwerveDrive(SOTA_SwerveModule[] modules, SwerveDriveKinematics driveKinematics, SOTA_Gyro gyro,
            SOTA_SwerveDriveConfig config) throws NullConfigException {
        this.modules = modules;
        this.mDriveKinematics = driveKinematics;
        this.mGyro = gyro;
        this.fieldCentric = false;

        if (config == null) {
            throw new NullConfigException("SwerveDrive: nullConfig");
        }

        Optional.ofNullable(config.getMaxSpeed()).ifPresent((maxSpeed) -> this.MAX_SPEED = maxSpeed);
        Optional.ofNullable(config.getMaxRotationalVelocity())
                .ifPresent((maxRttn) -> this.MAX_ROTATIONAL_VELOCITY = maxRttn);

        this.mDriveOdometry = new SwerveDriveOdometry(mDriveKinematics, mGyro.getRotation2d(), getModulePositions());
        AutoBuilder.configureHolonomic(this::getPose, this::resetPose, this::getRelativeSpeeds,
                this::robotRelativeDrive,
                new HolonomicPathFollowerConfig(Conversions.feetPerSecToMetersPerSec(MAX_SPEED),
                        config.getDriveBaseRadius(), new ReplanningConfig()),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, this);

        mField2d = new Field2d();
        this.sTab = Shuffleboard.getTab("Swerve");
        sTab.addNumber("Gyro Heading: ", mGyro::getAngle);
        sTab.addBoolean("FieldCentric Active: ", this::getFieldCentric);
        sTab.addNumber("Live Speed", () -> {return this.getRelativeSpeeds().omegaRadiansPerSecond;});
        sTab.add(mField2d);

    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];

        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }

        return positions;
    }

    /**
     * Drives the robot from joystick inputs
     * 
     * @param frwrd forward input
     * @param strf  side to side input
     * @param rttn  rotational input
     */
    public void drive(double frwrd, double strf, double rttn) {
        frwrd = MathUtil.clamp(frwrd, -1, 1) * Conversions.feetPerSecToMetersPerSec(MAX_SPEED);
        strf = MathUtil.clamp(strf, -1, 1) * Conversions.feetPerSecToMetersPerSec(MAX_SPEED);
        rttn = MathUtil.clamp(rttn, -1, 1) * MAX_ROTATIONAL_VELOCITY;
        drive(new ChassisSpeeds(frwrd, strf, rttn));
    }

    public void robotRelativeDrive(ChassisSpeeds robotRelative) {
        SwerveModuleState[] states = mDriveKinematics.toSwerveModuleStates(robotRelative);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Conversions.feetPerSecToMetersPerSec(MAX_SPEED));

        for (int i = 0; i < states.length; i++) {
            modules[i].setModule(states[i]);
        }
    }

    /**
     * Drive the robot from a ChassisSpeeds object
     * 
     * @param speeds the overall robot speeds
     */
    public void drive(ChassisSpeeds speeds) {
        if (fieldCentric) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, mGyro.getRotation2d());
        }
        
        SwerveModuleState[] states = mDriveKinematics.toSwerveModuleStates(speeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, Conversions.feetPerSecToMetersPerSec(MAX_SPEED));

        for (int i = 0; i < states.length; i++) {
            modules[i].setModule(states[i]);
        }
    }

    /**
     * Resets the heading of the robot
     */
    public void resetHeading() {
        mGyro.resetAngle();
        mDriveOdometry.resetPosition(mGyro.getRotation2d(), getModulePositions(), currentPose);
    }

    /**
     * @return fieldCentric status
     */
    public boolean getFieldCentric() {
        return fieldCentric;
    }

    /**
     * Set the state of field centric driving
     * 
     * @param fieldCentric true for enabled
     */
    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    @Override
    public void periodic() {
        currentPose = mDriveOdometry.update(mGyro.getRotation2d(), getModulePositions());
        mField2d.setRobotPose(currentPose);
    }

    private Pose2d getPose() {
        return currentPose;
    }

    public void resetPose(Pose2d newPose) {
        mDriveOdometry.resetPosition(mGyro.getRotation2d(), getModulePositions(), newPose);
    }

    private ChassisSpeeds getRelativeSpeeds() {
        return mDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

}