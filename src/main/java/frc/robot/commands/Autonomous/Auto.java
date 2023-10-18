// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ArmGrab.grabberPush;
import frc.robot.commands.ArmGrab.releasedPull;
import frc.robot.commands.ArmGrab.releasedPush;
import frc.robot.commands.ArmLift.ResetPosition;
import frc.robot.commands.ArmLift.armMove;
import frc.robot.commands.Automation.AutoBalance;
import frc.robot.commands.Drivetrain.Odometry;
import frc.robot.subsystems.ArmLift.positionStates;

public final class Auto {
  /** Example static factory for an autonomous command. */
  public static CommandBase Red1() {
    //return Commands.sequence(new Odometry(new Pose2d(new Translation2d(14.664, 0.5), new Rotation2d(0))), new Odometry(new Pose2d(new Translation2d(14.664, 2.159), new Rotation2d(0))));
    return Commands.sequence(new ResetPosition(),
                            new armMove(positionStates.PICKUP),
                            new WaitCommand(0.2), 
                            new grabberPush(),
                            new WaitCommand(0.4),
                            new armMove(positionStates.REST), 
                            new releasedPull(),
                            new Odometry(new Pose2d(14.974, 
                            0.5, Rotation2d.fromDegrees(-90))),
                            new Odometry(new Pose2d(10, 
                            0.5, Rotation2d.fromDegrees(-90))));
  }

  public static CommandBase Red2() {
    return Commands.sequence(new ResetPosition(),
                            new armMove(positionStates.PICKUP),
                            new WaitCommand(0.2), 
                            new grabberPush(),
                            new armMove(positionStates.REST), 
                            new releasedPull(),
                            new Odometry(new Pose2d(14.974, 
                            0.5, Rotation2d.fromDegrees(-90))),
                            new Odometry(new Pose2d(10, 
                            0.5, Rotation2d.fromDegrees(-90))));
                            // new armMove(positionStates.CHARGING_STATION),
                            // new armMove(positionStates.REST),
                            // new AutoBalance());
  }

  public static CommandBase Red3() {
    return Commands.sequence(new ResetPosition(),
                          new armMove(positionStates.PICKUP),
                          new WaitCommand(0.2), 
                          new grabberPush(),
                          new armMove(positionStates.REST), 
                          new releasedPull(),
                          new Odometry(new Pose2d(14.974, 
                          5.6, Rotation2d.fromDegrees(-90))),
                          new Odometry(new Pose2d(10, 
                          5.6, Rotation2d.fromDegrees(-90))));
  }

  public static CommandBase Blue1() {
    return Commands.sequence(new ResetPosition(),
                            new armMove(positionStates.PICKUP),
                            new WaitCommand(0.2), 
                            new grabberPush(),
                            new WaitCommand(0.4),
                            new armMove(positionStates.REST), 
                            new releasedPull(),
                            new Odometry(new Pose2d(1.551, 
                            0.5, Rotation2d.fromDegrees(-90))),
                            new Odometry(new Pose2d(6.525, 
                            0.5, Rotation2d.fromDegrees(-90))));
  }

  public static CommandBase Blue2() {
    return Commands.sequence(new ResetPosition(),
                            new armMove(positionStates.PICKUP),
                            new WaitCommand(0.2), 
                            new grabberPush(),
                            new WaitCommand(0.4),
                            new armMove(positionStates.REST), 
                            new releasedPull(),
                            new Odometry(new Pose2d(1.551, 
                            0.5, Rotation2d.fromDegrees(-90))),
                            new Odometry(new Pose2d(6.525, 
                            0.5, Rotation2d.fromDegrees(-90))));
  }
  

  public static CommandBase Blue3() {
    return Commands.sequence(new ResetPosition(),
                            new armMove(positionStates.PICKUP),
                            new WaitCommand(0.2), 
                            new grabberPush(),
                            new armMove(positionStates.REST), 
                            new releasedPull(),
                            new Odometry(new Pose2d(1.551, 
                            5.6, Rotation2d.fromDegrees(-90))),
                            new Odometry(new Pose2d(6.525, 
                            5.6, Rotation2d.fromDegrees(-90))));
  }

  public static CommandBase defaultCommand() {
    return Commands.sequence(new ResetPosition(), 
                            new armMove(positionStates.PICKUP),
                            new WaitCommand(0.2), 
                            new grabberPush(), 
                            new WaitCommand(0.4), 
                            new armMove(positionStates.REST));
  }

  private Auto() {

    throw new UnsupportedOperationException("This is a utility class!");
  }
}