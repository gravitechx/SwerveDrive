// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADXL345_I2C.Axes;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import frc.robot.auto.modes.DepTwoGrabOne;
//import frc.robot.auto.modes.ScoreTwo;
//import frc.robot.auto.modes.StraightToDepo;
//import frc.robot.auto.modes.TestAuto;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Instantiating our robot subsystem
  // private final Shooter shooter = new Shooter();
  // private final Feeder feed = new Feeder();
  // private final Intake intake = new Intake();
  // private final Climber climber = new Climber();
  // private final Limelight limelight = new Limelight(feed, intake);
  // private final Lights lights = new Lights();

  private final Joystick controller = new Joystick(Constants.OI.controllerPort);
  private final Joystick buttons = new Joystick(Constants.OI.buttonPanelPort);

  
  private final JoystickButton zeroGyro = new JoystickButton(controller, Constants.OI.resetGyro);
 
  
  
  private final Trigger trigger = new Trigger(() -> controller.getRawAxis(3) == 1);
  private final Trigger trigger2 = new Trigger(() -> controller.getRawAxis(3) == -1);

  // Swerve stuff
  private final Swerve s_Swerve = new Swerve();
  private final Mode modeControl = Mode.getInstance();

  private double motor1Speed = 1;
  private double motor2Speed = 1;
  // ^^ Shooter motor variables that are changed on the fly

  public RobotContainer() {
    configureAutoCommands();

    s_Swerve.setDefaultCommand(
      new TeleopSwerve(
        s_Swerve, 
        () -> controller.getRawAxis(Constants.OI.translationAxis),
        () -> controller.getRawAxis(Constants.OI.strafeAxis),
        () -> controller.getRawAxis(Constants.OI.rotationAxis),
        () -> false
      )
    );

    configureBindings();
  }

  private void configureAutoCommands() {
    //
    NamedCommands.registerCommand("FlipGyro", new InstantCommand(() -> s_Swerve.flipZeroGyro()));
  }

  private void configureBindings() {
    

    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
 
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return null;
    return AutoBuilder.buildAuto("Blue4NoteE");
    // s_Swerve.resetOdometry(PathPlannerPath.fromChoreoTrajectory("NewPath").getPreviewStartingHolonomicPose());
    // return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("NewPath"));
  }
}

