// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ArmLift.*;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ArmLift;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  //initialize the subsystem
  public static final ArmLift m_armLift = new ArmLift();
  public static final Joystick m_joystick = new Joystick(0);
  public static final Joystick m_opBoard = new Joystick(1);

  public static JoystickButton upArmButton = new JoystickButton(m_joystick,1);
  public static JoystickButton downArmButton = new JoystickButton(m_joystick, 2);
  // public static Joystick bottomNodeButton = new Joystick(4);
  // public static Joystick middleNodeButton = new Joystick(5);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
      upArmButton.whenHeld(new MoveArm(ArmLift.moveArmJoystick.Up));
      downArmButton.whenHeld(new MoveArm(ArmLift.moveArmJoystick.Down));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
