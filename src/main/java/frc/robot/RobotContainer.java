// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Automation.AutoBalance;
import frc.robot.commands.Autonomous.*;
import frc.robot.subsystems.*;
//import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // public static SendableChooser<Command> m_chooser = new SendableChooser<>();
  // ComplexWidget ShuffleBoardAutonomousRoutines = Shuffleboard.getTab("Driver").add("Autonoumous Routines Selector", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 2).withPosition(0, 2);
  
  public static final Drivetrain m_drivetrain = new Drivetrain();
  public static final Vision m_vision = new Vision();
  public static final ArmGrab m_grabber = new ArmGrab();
  public static final ArmLift m_armLift = new ArmLift();

  public static final Joystick swerveStick = new Joystick(0);
  public static final JoystickButton m_robotRelativeMode = new JoystickButton(swerveStick, 1);

  public static final Joystick operatorBoard = new Joystick(1);
  //public static JoystickButton autoBottom = new JoystickButton(operatorBoard, 1);
  public static JoystickButton autoMiddle = new JoystickButton(operatorBoard, 5);
  public static final JoystickButton m_resetGyro = new JoystickButton(operatorBoard, 7);

  public static JoystickButton downArmButton = new JoystickButton(operatorBoard, 9);
  public static JoystickButton upArmButton = new JoystickButton(operatorBoard, 10);

  public static final JoystickButton coneMode = new JoystickButton(operatorBoard, 11);

  public static final JoystickButton manualGrabClose = new JoystickButton(operatorBoard, 12);
  public static final JoystickButton autoBalance = new JoystickButton(operatorBoard, 13);
  public static final JoystickButton manualGrabOpen = new JoystickButton(operatorBoard, 14);

  public static final JoystickButton turningMode = new JoystickButton(operatorBoard, 15);

  //TODO: Change port number





  // Joystick

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
      m_chooser.setDefaultOption("Default", Auto.defaultCommand());
      m_chooser.addOption("Red1(ID 1)",  Auto.Red1());
      m_chooser.addOption("Red2(ID 2)", Auto.Red2());
      m_chooser.addOption("Red3(ID 3)", Auto.Red3());
      m_chooser.addOption("Blue1(ID 6)", Auto.Blue1());
      m_chooser.addOption("Blue2(ID 7) ", Auto.Blue2());
      m_chooser.addOption("Blue3(ID 8)", Auto.Blue3());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // autoMiddle.whenHeld(new AutomationMiddle());
    // autoBottom.whenHeld(new AutomationBottom());
    autoBalance.whileFalse(new AutoBalance());
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static SendableChooser<Command> m_chooser = new SendableChooser<>();
  ComplexWidget ShuffleBoardAutonomousRoutines = Shuffleboard.getTab("Prematch").add("Autonoumous Routines Selector", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 2).withPosition(0, 2);

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
