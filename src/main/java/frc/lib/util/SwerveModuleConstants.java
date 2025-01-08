package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.CTREConfigs;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    // public final Rotation2d angleOffset;
    public final CANcoderConfiguration individualModuleConfig;


    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, double magnetOffset, CANcoderConfiguration swerveCANcoderConfig) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.individualModuleConfig = swerveCANcoderConfig;
        this.individualModuleConfig.MagnetSensor.MagnetOffset = magnetOffset;
        // this.angleOffset = angleOffset;
    }
}
