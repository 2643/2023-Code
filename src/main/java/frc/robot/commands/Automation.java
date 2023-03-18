// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmGrab.GrabberClose;
import frc.robot.commands.ArmGrab.GrabberOpen;
import frc.robot.commands.ArmLift.armMove;
import frc.robot.commands.Drivetrain.Odometry;
import frc.robot.subsystems.ArmGrab.States;
import frc.robot.subsystems.ArmLift.positionStates;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Automation extends SequentialCommandGroup {
  /** Creates a new automations. */
  public Automation() {
    List<Pose2d> redConePoses = new ArrayList<>();
      redConePoses.add(new Pose2d(Constants.Position2d.FIRST_RED_TEAM_CONE_X_VALUE, Constants.Position2d.FIRST_RED_TEAM_CONE_Y_VALUE, new Rotation2d(0)));
      redConePoses.add(new Pose2d(Constants.Position2d.SECOND_RED_TEAM_CONE_X_VALUE, Constants.Position2d.SECOND_RED_TEAM_CONE_Y_VALUE, new Rotation2d(0)));
      redConePoses.add(new Pose2d(Constants.Position2d.THIRD_RED_TEAM_CONE_X_VALUE, Constants.Position2d.THIRD_RED_TEAM_CONE_Y_VALUE, new Rotation2d(0)));
      redConePoses.add(new Pose2d(Constants.Position2d.FOURTH_RED_TEAM_CONE_X_VALUE, Constants.Position2d.FOURTH_RED_TEAM_CONE_Y_VALUE, new Rotation2d(0)));
      redConePoses.add(new Pose2d(Constants.Position2d.FIFTH_RED_TEAM_CONE_X_VALUE, Constants.Position2d.FIFTH_RED_TEAM_CONE_Y_VALUE, new Rotation2d(0)));
      redConePoses.add(new Pose2d(Constants.Position2d.SIXTH_RED_TEAM_CONE_X_VALUE, Constants.Position2d.SIXTH_RED_TEAM_CONE_Y_VALUE, new Rotation2d(0)));

    redConePoses.add(new Pose2d(0, 0, new Rotation2d(0)));

    List<Pose2d> redCubePoses = new ArrayList<>();
    redCubePoses.add(new Pose2d(Constants.Position2d.FIRST_RED_TEAM_CUBE_X_VALUE, Constants.Position2d.FIRST_RED_TEAM_CUBE_Y_VALUE, new Rotation2d(0)));
    redCubePoses.add(new Pose2d(Constants.Position2d.SECOND_RED_TEAM_CUBE_X_VALUE, Constants.Position2d.SECOND_RED_TEAM_CUBE_Y_VALUE, new Rotation2d(0)));
    redCubePoses.add(new Pose2d(Constants.Position2d.THIRD_RED_TEAM_CUBE_X_VALUE, Constants.Position2d.THIRD_RED_TEAM_CUBE_Y_VALUE, new Rotation2d(0)));

    List<Pose2d> blueConePoses = new ArrayList<>();
    blueConePoses.add(new Pose2d(Constants.Position2d.FIRST_BLUE_TEAM_CONE_X_VALUE, Constants.Position2d.FIRST_BLUE_TEAM_CONE_Y_VALUE, new Rotation2d(0)));
    blueConePoses.add(new Pose2d(Constants.Position2d.SECOND_BLUE_TEAM_CONE_X_VALUE, Constants.Position2d.SECOND_BLUE_TEAM_CONE_Y_VALUE, new Rotation2d(0)));
    blueConePoses.add(new Pose2d(Constants.Position2d.THIRD_BLUE_TEAM_CONE_X_VALUE, Constants.Position2d.THIRD_BLUE_TEAM_CONE_Y_VALUE, new Rotation2d(0)));

    List<Pose2d> blueCubePoses = new ArrayList<>();
    blueCubePoses.add(new Pose2d(Constants.Position2d.FIRST_BLUE_TEAM_CONE_X_VALUE, Constants.Position2d.FIRST_BLUE_TEAM_CONE_Y_VALUE, new Rotation2d(0)));
    blueCubePoses.add(new Pose2d(Constants.Position2d.SECOND_BLUE_TEAM_CONE_X_VALUE, Constants.Position2d.SECOND_BLUE_TEAM_CONE_Y_VALUE, new Rotation2d(0)));
    blueCubePoses.add(new Pose2d(Constants.Position2d.THIRD_BLUE_TEAM_CONE_X_VALUE, Constants.Position2d.THIRD_BLUE_TEAM_CONE_Y_VALUE, new Rotation2d(0)));

    Pose2d redNearestConePose = new Pose2d();
    redNearestConePose.nearest(redConePoses);
    Pose2d redNearestCubePose = new Pose2d();
    redNearestCubePose.nearest(redCubePoses);

    Pose2d blueNearestConePose = new Pose2d();
    blueNearestConePose.nearest(blueConePoses);
    Pose2d blueNearestCubePose = new Pose2d();
    blueNearestCubePose.nearest(blueCubePoses);


  //List<Pose2d> poses = new ArrayList<>();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  if (DriverStation.getAlliance() == Alliance.Red){
    if (RobotContainer.m_grabber.getArmGrabState() != States.CLOSED){
       addCommands(new Odometry(new Pose2d(Constants.Position2d.RED_TEAM_PICKUP_X_VALUE, Constants.Position2d.RED_TEAM_PICKUP_Y_VALUE, new Rotation2d(0))), 
                  new armMove(positionStates.PICKUP), 
                  new WaitCommand(0), 
                  new GrabberClose());
    }
    else{
      if(RobotContainer.coneMode.getAsBoolean()) {
        addCommands(new Odometry(redNearestConePose), 
                    new armMove(positionStates.CONE), 
                    new WaitCommand(0), 
                    new GrabberOpen());
      } else {
        addCommands(new Odometry(redNearestConePose), 
                    new armMove(positionStates.CUBE), 
                    new WaitCommand(0), 
                    new GrabberOpen());
      }
    }
   
  } else if (DriverStation.getAlliance() == Alliance.Blue){
    if (RobotContainer.m_grabber.getArmGrabState() != States.CLOSED) {
      addCommands(new Odometry(new Pose2d(Constants.Position2d.BLUE_TEAM_PICKUP_X_VALUE, Constants.Position2d.BLUE_TEAM_PICKUP_Y_VALUE, new Rotation2d(0))), 
                  new armMove(positionStates.PICKUP), 
                  new WaitCommand(0), 
                  new GrabberClose());
    }
    else {
      if(RobotContainer.coneMode.getAsBoolean()) {
        addCommands(new Odometry(blueNearestConePose), 
                    new armMove(positionStates.CONE), 
                    new WaitCommand(0), 
                    new GrabberOpen());
      } else {
        addCommands(new Odometry(blueNearestConePose), 
                    new armMove(positionStates.CUBE), 
                    new WaitCommand(0), 
                    new GrabberOpen());
      }
    }
  } else
    addCommands(new WaitCommand(0));
  }

   
}