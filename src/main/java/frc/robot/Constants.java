// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
//import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  

  public static final double stickDeadband = 0.05;
  public static final double kTrueMaxSpeed = .25;
  public static double topSpeed = 1;
  public static int stopSide = 1;
  public static boolean primeShooter = false;
  public static int state = 0; // Disabled 0; Auto 1; Teleop 2;
 

  public static final class OI {
    public static final String driver = "Justin";

    public static final int controllerPort = (driver == "Justin") ? 0 : 2;
    public static final int buttonPanelPort = 1;

    public static final int testNormalButton = 1;
    public static final int testReverseButton = 2;
    public static final int rightClimberUpButtonPanel = 5;
    public static final int rightClimberDownButtonPanel = 6;
    public static final int leftClimberUpButtonPanel = 3;
    public static final int leftClimberDownButtonPanel = 4;

    public static final int translationAxis = (driver == "Justin") ? 1 : 1;
    public static final int strafeAxis = (driver == "Justin") ? 0 : 0;
    public static final int rotationAxis = (driver == "Justin") ? 2 : 4;

    public static final int longShotButton = (driver == "Justin") ? 30 : 2;
    public static final boolean dummy = true;
   
    public static final int shotButton = (driver == "Justin") ? 30 : 3;

    public static final int intakeButton = (driver == "Justin") ? 4 : 6; // 1
    public static final int feedButton = (driver == "Justin") ? 2 : 5; // 5
    public static final int shootButton = (driver == "Justin") ? 1 : 1; // 2
    public static final int shootSlowButton = (driver == "Justin") ? 4 : 4;
    public static final int leftClimberUpButton = (driver == "Justin") ? 7 : 3;
    public static final int leftClimberDownButton = (driver == "Justin") ? 9 : 4;
    public static final int rightClimberUpButton = (driver == "Justin") ? 8 : 1;
    public static final int rightClimberDownButton = (driver == "Justin") ? 10 : 2;
    public static final int primeShooterButton = 20; // 3
    public static final int bothClimberUpButton = 20; //Make it an acutal button
    public static final int bothClimberDownButton = 20; //Make this an actual button
    // public static final int increaseTopSpeed = 6;
    //public static final int decreaseTopSpeed = 4;
    public static final int POVNorth = 0;
    public static final int POVSouth = 180;
    public static final int POVEast = 90;
    public static final int POVWest = 270;
    public static final int reverseFeed = (driver == "Justin") ? 12 : 12;
    public static final int resetGyro = (driver == "Justin") ? 11 : 11;

    //public static final int musicStartButton = 7;
    //public static final int musicStopButton = 8;
  }

  public static final class Lights {
    public static final int port = 9;
    public static final int length = 120;
  }

  public static final class Shooter {
    public static final int kMotorPortTop = 3; //NOT FINAL
    public static final int kMotorPortBottom = 2; //NOT FINAL
  }

  public static final class Intake {
    public static final int motorPort = 1;//NOT FINAL
    public static final int sensorPort = 0;
    public static final int sensorPort1 = 1;
  }

  public static final class Feeder {
    public static final int leftMotorPort = 4; //NOT FINAL
    public static final int rightMotorPort = 6; //NOT FINAL
    public static final int sensorPort = 1;
  }

  public static final class Climber {
    public static final int motorPort1 = 9;
    public static final int motorPort2 = 10;
  }

  public static final class Swerve {
    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);
        // .8, .93; .46, .48
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18.25); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(19); //TODO: This must be tuned to specific robot
        public static final double swerveRadius = Units.inchesToMeters(Math.sqrt(Math.pow(19 / 2, 2) + Math.pow(18.25 / 2, 2)));
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        // public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        // public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;
        public static final InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;

        /* Angle Encoder Invert */
        // public static final boolean canCoderInvert = chosenModule.canCoderInvert;
        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = .25;
        public static final double closedLoopRamp = 0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 1; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 10; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7; //7
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0); // 177.978
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-0.232910*360); // 101.898
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0); // 101.25
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0); // 151.171
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 6.16; // 6.16
        public static final double kMaxAccelerationMetersPerSecondSquared = 5; // 4.5
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final PIDController kPXController = new PIDController(1, 0, 0);
        public static final PIDController kPYController = new PIDController(1, 0, 0);
        public static final double kPThetaController = 1;
    
        public static final TrajectoryConfig trajConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond,
            kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(Swerve.swerveKinematics);

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final ProfiledPIDController kThetaController = new ProfiledPIDController(1, 0, 0, kThetaControllerConstraints);
        {
            kThetaController.enableContinuousInput(-Math.PI, Math.PI);
        }
}

}
