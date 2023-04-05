// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ArmGrab.*;
import frc.robot.commands.ArmLift.*;
import frc.robot.commands.Automation.AutomationMiddle;
// import frc.robot.commands.Drivetrain.Odometry;
import frc.robot.commands.Drivetrain.SwerveDrive;
// import frc.robot.subsystems.ArmGrab.States;
import frc.robot.subsystems.ArmLift.ArmLiftStates;
import frc.robot.subsystems.ArmLift.moveArmJoystick;
// import frc.robot.subsystems.ArmLift.positionStates;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  Field2d field = new Field2d();
  boolean autoMiddleMode = false;


  GenericEntry timeEntry = Shuffleboard.getTab("Driver").add("Time", 0).withPosition(0, 0).withSize(2, 2).getEntry();
  GenericEntry pickupPlaceEntry = Shuffleboard.getTab("Driver").add("Pickup(T) or Place(F)", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 2).withSize(2, 2).getEntry();
  GenericEntry joystickConnectedEntry = Shuffleboard.getTab("Driver").add("Joystick Connected", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(4, 0).withSize(2, 1).getEntry();
  GenericEntry opBoardConnectedEntry = Shuffleboard.getTab("Driver").add("OpBoard Connected", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(4, 2).withSize(2, 1).getEntry();
  GenericEntry coneCubeEntry = Shuffleboard.getTab("Driver").add("Cone(T) or Cube(F)", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 0).withSize(2, 2).getEntry();
  public static GenericEntry aprilTagsDetection = Shuffleboard.getTab("Driver").add("Detects AprilTags", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 2).withSize(2, 2).getEntry();

  //GenericEntry chargeStationEntry = Shuffleboard.getTab("Driver").add("Charge Station", true).withWidget(BuiltInWidgets.kToggleButton).withPosition(9, 2).withSize(2, 1).getEntry();
  //ComplexWidget cameraShuffleboard = Shuffleboard.getTab("Driver").addCamera("Camera", "Limelight", "http://10.26.43.17:5800/").withPosition(19, 0).withSize(5, 5);
  ComplexWidget fieldShuffleboard = Shuffleboard.getTab("Driver").add("2023-Field", field).withWidget(BuiltInWidgets.kField).withProperties(Map.of("robot icon size", 35)).withPosition(4, 0).withSize(8, 5);

  // GenericEntry dock = Shuffleboard.getTab("Driver").add("dock", true).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
  // GenericEntry chargeStation = Shuffleboard.getTab("Driver").add("Charge Station", true).withWidget(BuiltInWidgets.kToggleButton).getEntry();


  private static enum AUTOMATION_STATES {
    RESET,
    NOT_INITIALIZED,
    INITIALIZING,
    FIRST_PICKUP_MOVE,
    SECOND_PICKUP_MOVE,
    ARM_MOVE_PICKUP,
    PICKUP_END,
    DROP_MOVE,
    DROP_LOWER,
    DROP_OPEN,
    DROP_END
  }

  private static enum SECOND_AUTO_SOLUTION {
    RESET,
    NOT_INITIALIZED,
    RUNNING, 
    CALLED
  }

  public static AUTOMATION_STATES currentAutoState = AUTOMATION_STATES.NOT_INITIALIZED;
  public static SECOND_AUTO_SOLUTION currentSecondAutoState = SECOND_AUTO_SOLUTION.NOT_INITIALIZED;

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
  double autoAprilTag;
  @Override
  public void autonomousInit() {
    autoAprilTag = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(-1);

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
    CommandScheduler.getInstance().setDefaultCommand(RobotContainer.m_drivetrain, new SwerveDrive());
    // redConePoses = new ArrayList<>();
    // redConePoses.add(new Pose2d(Constants.Position2d.FIRST_RED_TEAM_CONE_X_VALUE, Constants.Position2d.FIRST_RED_TEAM_CONE_Y_VALUE, new Rotation2d(-90)));
    // redConePoses.add(new Pose2d(Constants.Position2d.SECOND_RED_TEAM_CONE_X_VALUE, Constants.Position2d.SECOND_RED_TEAM_CONE_Y_VALUE, new Rotation2d(-90)));
    // redConePoses.add(new Pose2d(Constants.Position2d.THIRD_RED_TEAM_CONE_X_VALUE, Constants.Position2d.THIRD_RED_TEAM_CONE_Y_VALUE, new Rotation2d(-90)));
    // redConePoses.add(new Pose2d(Constants.Position2d.FOURTH_RED_TEAM_CONE_X_VALUE, Constants.Position2d.FOURTH_RED_TEAM_CONE_Y_VALUE, new Rotation2d(-90)));
    // redConePoses.add(new Pose2d(Constants.Position2d.FIFTH_RED_TEAM_CONE_X_VALUE, Constants.Position2d.FIFTH_RED_TEAM_CONE_Y_VALUE, new Rotation2d(-90)));
    // redConePoses.add(new Pose2d(Constants.Position2d.SIXTH_RED_TEAM_CONE_X_VALUE, Constants.Position2d.SIXTH_RED_TEAM_CONE_Y_VALUE, new Rotation2d(-90)));

    // redCubePoses = new ArrayList<>();
    // redCubePoses.add(new Pose2d(Constants.Position2d.FIRST_RED_TEAM_CUBE_X_VALUE, Constants.Position2d.FIRST_RED_TEAM_CUBE_Y_VALUE, new Rotation2d(-90)));
    // redCubePoses.add(new Pose2d(Constants.Position2d.SECOND_RED_TEAM_CUBE_X_VALUE, Constants.Position2d.SECOND_RED_TEAM_CUBE_Y_VALUE, new Rotation2d(-90)));
    // redCubePoses.add(new Pose2d(Constants.Position2d.THIRD_RED_TEAM_CUBE_X_VALUE, Constants.Position2d.THIRD_RED_TEAM_CUBE_Y_VALUE, new Rotation2d(-90)));

    // blueConePoses = new ArrayList<>();
    // blueConePoses.add(new Pose2d(Constants.Position2d.FIRST_BLUE_TEAM_CONE_X_VALUE, Constants.Position2d.FIRST_BLUE_TEAM_CONE_Y_VALUE, new Rotation2d(90)));
    // blueConePoses.add(new Pose2d(Constants.Position2d.SECOND_BLUE_TEAM_CONE_X_VALUE, Constants.Position2d.SECOND_BLUE_TEAM_CONE_Y_VALUE, new Rotation2d(90)));
    // blueConePoses.add(new Pose2d(Constants.Position2d.THIRD_BLUE_TEAM_CONE_X_VALUE, Constants.Position2d.THIRD_BLUE_TEAM_CONE_Y_VALUE, new Rotation2d(90)));
    // blueConePoses.add(new Pose2d(Constants.Position2d.FOURTH_BLUE_TEAM_CONE_X_VALUE, Constants.Position2d.FOURTH_BLUE_TEAM_CONE_Y_VALUE, new Rotation2d(90)));
    // blueConePoses.add(new Pose2d(Constants.Position2d.FIFTH_BLUE_TEAM_CONE_X_VALUE, Constants.Position2d.FIFTH_BLUE_TEAM_CONE_Y_VALUE, new Rotation2d(90)));
    // blueConePoses.add(new Pose2d(Constants.Position2d.SIXTH_BLUE_TEAM_CONE_X_VALUE, Constants.Position2d.SIXTH_BLUE_TEAM_CONE_Y_VALUE, new Rotation2d(90)));

    // blueCubePoses = new ArrayList<>();
    // blueCubePoses.add(new Pose2d(Constants.Position2d.FIRST_BLUE_TEAM_CUBE_X_VALUE, Constants.Position2d.FIRST_BLUE_TEAM_CUBE_Y_VALUE, new Rotation2d(90)));
    // blueCubePoses.add(new Pose2d(Constants.Position2d.SECOND_BLUE_TEAM_CUBE_X_VALUE, Constants.Position2d.SECOND_BLUE_TEAM_CUBE_Y_VALUE, new Rotation2d(90)));
    // blueCubePoses.add(new Pose2d(Constants.Position2d.THIRD_BLUE_TEAM_CUBE_X_VALUE, Constants.Position2d.THIRD_BLUE_TEAM_CUBE_Y_VALUE, new Rotation2d(90)));

  }

  Command firstMoveCommand;
  Command moveCommand;
  Command moveArmCommand;
  Command moveGrabber;

  Command secondSolution;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // if(RobotContainer.m_armLift.getArmLiftState() == ArmLiftStates.INITIALIZING_CALLED) {
      
    //   //RobotContainer.m_armLift.setArmLiftState(ArmLiftStates.INITIALIZING);
    // }

    if(!RobotContainer.autoMiddle.getAsBoolean()) {
      // RobotContainer.autoMiddle.whileTrue(new AutomationMiddle());
      // RobotContainer.autoBottom.whileTrue(new AutomationBottom());
      if(RobotContainer.m_armLift.getArmLiftState() == ArmLiftStates.INITIALIZED) {
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
   
      if(RobotContainer.m_grabber.isClosed()) {
        pickupPlaceEntry.setBoolean(false);
      } else {
        pickupPlaceEntry.setBoolean(true);
      }

      if(RobotContainer.manualGrabClose.getAsBoolean()) {
        CommandScheduler.getInstance().schedule(new GrabberClose());
      } else if(RobotContainer.manualGrabOpen.getAsBoolean()) {
        CommandScheduler.getInstance().schedule(new GrabberOpen());
      }
    }

    if(RobotContainer.m_armLift.getArmLiftState() == ArmLiftStates.INITIALIZED) {
      switch (currentSecondAutoState) {
        case RESET:
          secondSolution.cancel();
          currentSecondAutoState = SECOND_AUTO_SOLUTION.NOT_INITIALIZED;
          break;
        case NOT_INITIALIZED:
          if(RobotContainer.autoMiddle.getAsBoolean() && RobotContainer.m_vision.hasTarget.getDouble(0) == 1) {
            currentSecondAutoState = SECOND_AUTO_SOLUTION.CALLED;
          }
          break;
        case CALLED:
          secondSolution = new AutomationMiddle();
          secondSolution.schedule();
          currentSecondAutoState = SECOND_AUTO_SOLUTION.RUNNING;
          break;
        case RUNNING:
          if(!RobotContainer.autoMiddle.getAsBoolean()) {
            currentSecondAutoState =  SECOND_AUTO_SOLUTION.RESET;
          }
          break;
        default:
          System.out.println("Failure");
          break;
      }
    }

    // if(RobotContainer.autoMiddle.getAsBoolean() && !autoMiddleMode) {
    //   CommandScheduler.getInstance().schedule(new AutomationMiddle());
    //   autoMiddleMode = true;
    // } 
    
    // if(!RobotContainer.autoMiddle.getAsBoolean() && autoMiddleMode) {
    //   CommandScheduler.getInstance().cancelAll();
    //   autoMiddleMode = false;
    // }

    // if(!RobotContainer.autoMiddle.getAsBoolean() && currentAutoState != AUTOMATION_STATES.NOT_INITIALIZED) {
    //   firstMoveCommand.cancel();
    //   moveCommand.cancel();
    //   moveArmCommand.cancel();
    //   moveGrabber.cancel();
    // }

    // if(!RobotContainer.autoMiddle.getAsBoolean()) {
    //   currentAutoState = AUTOMATION_STATES.NOT_INITIALIZED;
    // }

    // switch(currentAutoState) {
    //   case RESET:
    //     if(!RobotContainer.autoMiddle.getAsBoolean())
    //       currentAutoState = AUTOMATION_STATES.NOT_INITIALIZED;
    //     break;
    //   case NOT_INITIALIZED:
    //     if(RobotContainer.autoMiddle.getAsBoolean())
    //       currentAutoState = AUTOMATION_STATES.INITIALIZING;
    //     break;
    //   case INITIALIZING:
    //     redNearestConePose = RobotContainer.m_drivetrain.getPose();
    //     redNearestCubePose = RobotContainer.m_drivetrain.getPose();
    //     blueNearestConePose = RobotContainer.m_drivetrain.getPose();
    //     blueNearestCubePose = RobotContainer.m_drivetrain.getPose();

    //     redNearestConePose.nearest(redConePoses);
    //     redNearestCubePose.nearest(redCubePoses);
    //     blueNearestConePose.nearest(blueConePoses);
    //     blueNearestCubePose.nearest(blueCubePoses);
    //     if(RobotContainer.m_grabber.isClosed()) {
    //       moveCommand = new Odometry(new Pose2d());
    //       currentAutoState = AUTOMATION_STATES.DROP_MOVE;
    //     } else {
    //       firstMoveCommand = new Odometry(new Pose2d());
    //       moveCommand = new Odometry(new Pose2d());
    //       currentAutoState = AUTOMATION_STATES.FIRST_PICKUP_MOVE;
    //     }
    //     break;
    //   case DROP_MOVE:
    //   if(moveCommand.isFinished()) {
    //     moveArmCommand = new armMove(positionStates.REST);
    //     currentAutoState = AUTOMATION_STATES.DROP_LOWER;
    //   }
    //     if(DriverStation.getAlliance() == Alliance.Blue) {
    //       if(!moveCommand.isScheduled()) {
    //         if(RobotContainer.coneMode.getAsBoolean()) {
    //           moveCommand = new Odometry(blueNearestConePose);
    //         } else {
    //           moveCommand = new Odometry(blueNearestCubePose);
    //         }
    //         CommandScheduler.getInstance().schedule(moveCommand);
    //       }
    //     } else {
    //       if(!moveCommand.isScheduled()) {
    //         if(RobotContainer.coneMode.getAsBoolean()) {
    //           moveCommand = new Odometry(redNearestConePose);
    //         } else {
    //           moveCommand = new Odometry(redNearestCubePose);
    //         }
    //           CommandScheduler.getInstance().schedule(moveCommand);
    //       }
    //     }
    //     break;
    //   case DROP_LOWER:
    //     if(moveArmCommand.isFinished()) {
    //       moveGrabber = new GrabberOpen();
    //       currentAutoState = AUTOMATION_STATES.DROP_OPEN;
    //     }

    //     if(!moveArmCommand.isScheduled()) {
    //       if(RobotContainer.coneMode.getAsBoolean()) {
    //         moveArmCommand = new armMove(positionStates.CONE);
    //       } else {
    //         moveArmCommand = new armMove(positionStates.CUBE);
    //       }
    //       CommandScheduler.getInstance().schedule(moveArmCommand);
    //     }
    //     break;
    //   case DROP_OPEN:
    //     if(moveGrabber.isFinished() && RobotContainer.m_grabber.getArmGrabState()== States.OPENED) {
    //       currentAutoState = AUTOMATION_STATES.DROP_END;
    //     }
    //     if(!moveGrabber.isScheduled() && RobotContainer.m_grabber.getArmGrabState() != States.OPENED) {
    //       CommandScheduler.getInstance().schedule(moveGrabber);
    //     }
    //     break;
    //   case DROP_END:
    //     moveArmCommand = new armMove(positionStates.REST);
    //     CommandScheduler.getInstance().schedule(moveArmCommand);
    //     currentAutoState = AUTOMATION_STATES.RESET;
    //     break;
    //   case FIRST_PICKUP_MOVE:
    //     if(firstMoveCommand.isFinished()) {
    //       moveArmCommand = new armMove(positionStates.REST);
    //       currentAutoState = AUTOMATION_STATES.ARM_MOVE_PICKUP;
    //     }

    //     if(!firstMoveCommand.isScheduled()) {
    //       if(DriverStation.getAlliance() == Alliance.Blue) {
    //         firstMoveCommand = new Odometry(new Pose2d(Constants.Position2d.STARTING_BLUE_PICKUP_X_VALUE, Constants.Position2d.STARTING_BLUE_PICKUP_Y_VALUE, Rotation2d.fromDegrees(-90)));
    //         CommandScheduler.getInstance().schedule(firstMoveCommand);
    //       } else {
    //         firstMoveCommand = new Odometry(new Pose2d(Constants.Position2d.STARTING_RED_PICKUP_X_VALUE, Constants.Position2d.STARTING_RED_PICKUP_Y_VALUE, Rotation2d.fromDegrees(90)));
    //       }
    //     }
    //     break;
    //   case ARM_MOVE_PICKUP:
    //   if(moveArmCommand.isFinished()) {
    //     moveGrabber = new GrabberOpen();
    //     moveArmCommand = new armMove(positionStates.REST);
    //     currentAutoState = AUTOMATION_STATES.SECOND_PICKUP_MOVE;
    //   }

    //   if(!moveArmCommand.isScheduled()) {
    //     if(RobotContainer.coneMode.getAsBoolean()) {
    //       moveArmCommand = new armMove(positionStates.CONE);
    //     } else {
    //       moveArmCommand = new armMove(positionStates.CUBE);
    //     }
    //     CommandScheduler.getInstance().schedule(moveArmCommand);
    //   }
    //     break;
    //   case SECOND_PICKUP_MOVE:
    //     if(moveCommand.isFinished()) {
    //       moveGrabber = new GrabberClose();
    //       currentAutoState = AUTOMATION_STATES.PICKUP_END;
    //     }

    //     if(!moveCommand.isScheduled()) {
    //       CommandScheduler.getInstance().schedule(moveGrabber);
    //       if(DriverStation.getAlliance() == Alliance.Blue) {
    //         moveCommand = new Odometry(new Pose2d(Constants.Position2d.BLUE_TEAM_PICKUP_X_VALUE, Constants.Position2d.BLUE_TEAM_PICKUP_Y_VALUE, Rotation2d.fromDegrees(-90)));
    //         CommandScheduler.getInstance().schedule(moveCommand);
    //       } else {
    //         moveCommand = new Odometry(new Pose2d(Constants.Position2d.RED_TEAM_PICKUP_X_VALUE, Constants.Position2d.RED_TEAM_PICKUP_Y_VALUE, Rotation2d.fromDegrees(90)));
    //         CommandScheduler.getInstance().schedule(moveCommand);
    //       }
    //     }
    //     break;
    //   case PICKUP_END:
    //     if(!moveGrabber.isScheduled()) {
    //       moveGrabber = new GrabberClose();
    //       CommandScheduler.getInstance().schedule(moveGrabber);
    //     }
    //     if(RobotContainer.m_grabber.getArmGrabState() == States.CLOSED) {
    //       if(DriverStation.getAlliance() == Alliance.Blue)
    //         firstMoveCommand = new Odometry(new Pose2d(Constants.Position2d.STARTING_BLUE_PICKUP_X_VALUE, Constants.Position2d.STARTING_BLUE_PICKUP_Y_VALUE, Rotation2d.fromDegrees(-90)));
    //       else
    //         firstMoveCommand = new Odometry(new Pose2d(Constants.Position2d.STARTING_RED_PICKUP_X_VALUE, Constants.Position2d.STARTING_RED_PICKUP_Y_VALUE, Rotation2d.fromDegrees(-90)));
    //       CommandScheduler.getInstance().schedule(firstMoveCommand);
    //       CommandScheduler.getInstance().schedule(new armMove(positionStates.REST));
    //       currentAutoState = AUTOMATION_STATES.RESET;
    //     }
    //     break;
    //   default:
    //     System.out.println("Code is trash");
    //     break;
    // }

    // if(RobotContainer.autoMiddle.getAsBoolean()) {
    //   //secondSolution = 
    // }

    
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
