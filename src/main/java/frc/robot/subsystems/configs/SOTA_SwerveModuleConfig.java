package frc.robot.subsystems.configs;

import SOTAlib.Config.CompositeMotorConfig;
import SOTAlib.Config.MotorControllerConfig;

public class SOTA_SwerveModuleConfig {
    
    /**
     * Name of the module for debugging
     */
    private String moduleName;

    /**
     * Composite Motor config for the angle motor and encoder 
     */
    private CompositeMotorConfig angleSystem;

    /**
     * speed motor config
     */
    private MotorControllerConfig speedConfig;

    /**
     * @return name of the module
     */
    public String getModuleName() {
        return moduleName;
    }

    /**
     * @return angle composite config
     */
    public CompositeMotorConfig getAngleSystem() {
        return this.angleSystem;
    }

    /**
     * @return config for the speed motor
     */
    public MotorControllerConfig getSpeedConfig() {
        return this.speedConfig;
    }
}
