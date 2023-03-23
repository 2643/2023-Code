// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Autonomous.Routine1;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static final String LimelightCameraL = null;
    private Command m_autonomousCommand;
  GenericEntry targetRobotX = Shuffleboard.getTab("Odometry").add("Robot-X", 0).getEntry();
  GenericEntry targetRobotY = Shuffleboard.getTab("Odometry").add("Robot-Y", 0).getEntry();
  GenericEntry targetRobotTurn = Shuffleboard.getTab("Odometry").add("Turn Angle", 0).getEntry();

  //Laptop 

  
  GenericEntry timeEntry = Shuffleboard.getTab("Sample").add("Time", 0).getEntry();
  GenericEntry dockEntry = Shuffleboard.getTab("Sample").add("dock", true).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
  GenericEntry chargeStationEntry = Shuffleboard.getTab("Sample").add("Charge Station", true).withWidget(BuiltInWidgets.kToggleButton).getEntry();
  GenericEntry apriltagsEntry= Shuffleboard.getTab("Sample").add("Detects Apriltags", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
  GenericEntry PickupPlaceEntry = Shuffleboard.getTab("Sample").add("Pickup or Place", false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
  GenericEntry OkToDropEntry = Shuffleboard.getTab("Sample").add("Pickup or Place", false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
  public static SendableChooser<Command> m_chooser = new SendableChooser<>();
  ComplexWidget autoEndLocationEntry = Shuffleboard.getTab("Sample").add("Autonomous End Location Selector", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);
 
  public final Command m_location1 = new Routine1(Constants.AUTONOMOUS_ENDING_LOCATION_ONE);
  public final Command m_location2 = new Routine1(Constants.AUTONOMOUS_ENDING_LOCATION_TWO);
  public final Command m_location3 = new Routine1(Constants.AUTONOMOUS_ENDING_LOCATION_THREE);
  public final Command m_location4 = new Routine1(Constants.AUTONOMOUS_ENDING_LOCATION_FOUR);
  m_chooser.setDefaultOption("First", m_location1);
  m_chooser.addOption("Second", m_location2);
  m_chooser.addOption("Third", m_location3);
  m_chooser.addOption("Fourth", m_location4);
  m_chooser.addOption("First", m_location1);
  
  



  
  //monitor
  //GenericEntry poseVision.getEstimatedPosition()
  
  ComplexWidget fieldShuffleboard = Shuffleboard.getTab("Field-New").add("2023-Field", Vision.getEstimatedPosition()).withWidget(BuiltInWidgets.kField).withProperties(Map.of("robot icon size", 20));
  
  


  private RobotContainer m_robotContainer;

   Object driverTab;
}
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
 
    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    HttpCamera limelightFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg%22");
    Sendable camera;
    ComplexWidget driverEntry = Shuffleboard.getTab("Sample").add("limelightCamera", limelightFeed).withPosition(0, 0).withSize(15, 8).withProperties(Map.of("Show Crosshair", true, "Show Controls", false));
    CameraServer camerServer;
    camerServer.startAutomaticCapture(0);
    CommandScheduler.getInstance().run();
    CommandScheduler.getInstance().run();
    m_robotContainer = new RobotContainer();
  }
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    // MjpegServer server;
    // HttpCamera LLFeed;
    //  UsbCamera cargoCam;
    //  int cameraStream = 0;
    //   HttpCamera limelightFeed;
  }

  public void Vision() {
      // ShuffleboardTab dashboardTab = Shuffleboard.getTab("Dash");
      // HttpCamera LLFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg%22");
      // UsbCamera cargoCam = CameraServer.startAutomaticCapture(0);
      // cargoCam.setConnectVerbose(0);
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
  public void autonomousPeriodic() {
  
  }

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
  public void simulationPeriodic() {

  }