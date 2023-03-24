// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ArmLift.MoveArm;
import frc.robot.commands.ArmLift.ResetPosition;
import frc.robot.commands.Drivetrain.SwerveDrive;
import frc.robot.subsystems.ArmGrab.States;
import frc.robot.subsystems.ArmLift.ArmLiftStates;
import frc.robot.subsystems.ArmLift.moveArmJoystick;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  Field2d field = new Field2d();

  GenericEntry timeEntry = Shuffleboard.getTab("Driver").add("Time", 0).withPosition(0, 0).withSize(2, 2).getEntry();
  GenericEntry targetRobotX = Shuffleboard.getTab("Driver").add("Robot-X", 0).withPosition(0, 2).withSize(2, 2).getEntry();
  GenericEntry targetRobotY = Shuffleboard.getTab("Driver").add("Robot-Y", 0).withPosition(2, 2).withSize(2, 2).getEntry();
  GenericEntry directionToMove = Shuffleboard.getTab("Driver").add("Directions to Move", "Not Complete").withPosition(2, 0).withSize(2, 2).getEntry();
  GenericEntry OkToDropEntry = Shuffleboard.getTab("Driver").add("Can Drop Item(T)", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(5, 2).withSize(2, 2).getEntry();
  GenericEntry pickupPlaceEntry = Shuffleboard.getTab("Driver").add("Pickup(T) or Place(F)", false).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(5, 2).withSize(2, 2).getEntry();
  GenericEntry joystickConnectedEntry = Shuffleboard.getTab("Driver").add("Joystick Connected", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(7, 0).withSize(2, 1).getEntry();
  GenericEntry opBoardConnectedEntry = Shuffleboard.getTab("Driver").add("OpBoard Connected", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(7, 1).withSize(2, 1).getEntry();
  GenericEntry coneCubeEntry = Shuffleboard.getTab("Driver").add("Cone(T) or Cube(F)", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(7, 3).withSize(2, 1).getEntry();

  GenericEntry chargeStationEntry = Shuffleboard.getTab("Driver").add("Charge Station", true).withWidget(BuiltInWidgets.kToggleButton).withPosition(9, 2).withSize(2, 1).getEntry();
  ComplexWidget cameraShuffleboard = Shuffleboard.getTab("Driver").addCamera("Camera", "Limelight", "http://10.26.43.17:5800/").withPosition(19, 0).withSize(5, 5);
  ComplexWidget fieldShuffleboard = Shuffleboard.getTab("Driver").add("2023-Field", field).withWidget(BuiltInWidgets.kField).withProperties(Map.of("robot icon size", 40)).withPosition(12, 0).withSize(7, 5);

  // GenericEntry dock = Shuffleboard.getTab("Driver").add("dock", true).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
  // GenericEntry chargeStation = Shuffleboard.getTab("Driver").add("Charge Station", true).withWidget(BuiltInWidgets.kToggleButton).getEntry();

  
  public static GenericEntry aprilTagsDetection = Shuffleboard.getTab("Driver").add("Detects Apriltags", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 0).withSize(2, 1).getEntry();
  //GenericEntry pickup_place = Shuffleboard.getTab("Driver").add("Pickup(T)/Place(F)", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
  public static SendableChooser<Command> m_chooser = new SendableChooser<>();
  ComplexWidget autoEndLocationEntry = Shuffleboard.getTab("Driver").add("Autonomous End Location Selector", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);
 
  // public final Command m_location1 = new Routine1(Constants.AUTONOMOUS_ENDING_LOCATION_ONE);
  // public final Command m_location2 = new Routine1(Constants.AUTONOMOUS_ENDING_LOCATION_TWO);
  // public final Command m_location3 = new Routine1(Constants.AUTONOMOUS_ENDING_LOCATION_THREE);
  // public final Command m_location4 = new Routine1(Constants.AUTONOMOUS_ENDING_LOCATION_FOUR);



  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // m_chooser.setDefaultOption("First", m_location1);
    // m_chooser.addOption("Second", m_location2);
    // m_chooser.addOption("Third", m_location3);
    // m_chooser.addOption("Fourth", m_location4);
    // m_chooser.addOption("First", m_location1);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    field.setRobotPose(RobotContainer.m_drivetrain.getPose().getX(), RobotContainer.m_drivetrain.getPose().getY(), RobotContainer.m_drivetrain.getPose().getRotation().minus(new Rotation2d(Math.PI/2)));
    timeEntry.setDouble(DriverStation.getMatchTime());
    if(RobotContainer.swerveStick.isConnected()) {
      joystickConnectedEntry.setBoolean(true);
    } else {
      joystickConnectedEntry.setBoolean(false);
    }

    if(RobotContainer.operatorBoard.isConnected()) {
      opBoardConnectedEntry.setBoolean(true);
    } else {
      opBoardConnectedEntry.setBoolean(false);
    }

    if(RobotContainer.coneMode.getAsBoolean()) {
      coneCubeEntry.setBoolean(true);
    } else {
      coneCubeEntry.setBoolean(false);
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().setDefaultCommand(RobotContainer.m_drivetrain, new SwerveDrive());
    if(RobotContainer.m_reset.getAsBoolean()) {
      RobotContainer.m_drivetrain.resetGyro();
    }

    if(RobotContainer.m_armLift.getArmLiftState() == ArmLiftStates.INITIALIZING_CALLED) {
      CommandScheduler.getInstance().schedule(new ResetPosition());
    }

    if(RobotContainer.m_armLift.getArmLiftState() == ArmLiftStates.NOT_INITIALIZED || RobotContainer.m_armLift.getArmLiftState() == ArmLiftStates.INITIALIZED) {
      if(RobotContainer.upArmButton.getAsBoolean()) {
        CommandScheduler.getInstance().schedule(new MoveArm(moveArmJoystick.Up));
      } else if (RobotContainer.downArmButton.getAsBoolean()) {
        CommandScheduler.getInstance().schedule(new MoveArm(moveArmJoystick.Down));
      } else {
        if(RobotContainer.m_armLift.getArmLiftState() == ArmLiftStates.INITIALIZED) {
          if(RobotContainer.m_armLift.changedEncoderPlacement()) {
            RobotContainer.m_armLift.setChangedEncoderPlacement(false);
            CommandScheduler.getInstance().schedule(new MoveArm(moveArmJoystick.Encoder));
          }
        }
      }
    }

    if(RobotContainer.m_grabber.getArmGrabState() == States.CLOSED) {
      pickupPlaceEntry.setBoolean(true);
    } else {
      pickupPlaceEntry.setBoolean(false);
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
