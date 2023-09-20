// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Testing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.commands.ArmGrab.GrabberClose;
// import frc.robot.commands.ArmGrab.GrabberOpen;
import frc.robot.commands.ArmLift.armMove;
import frc.robot.commands.Drivetrain.Odometry;
import frc.robot.subsystems.ArmLift.positionStates;

public final class Testing {
  /** Example static factory for an autonomous command. */
  public static CommandBase Routine1() {
    //return Commands.sequence(new Odometry(new Pose2d(new Translation2d(14.664, 0.5), new Rotation2d(0))), new Odometry(new Pose2d(new Translation2d(14.664, 2.159), new Rotation2d(0))));
    //return Commands.sequence(new Odometry(new Pose2d(new Translation2d(14.49, 2.65), Rotation2d.fromDegrees(-90))).raceWith(new WaitCommand(4)), new armMove(positionStates.CUBE), new WaitCommand(3), new GrabberOpen(), new WaitCommand(1), new armMove(positionStates.REST));
    return Commands.sequence( new WaitCommand(0));
  }

  // public static CommandBase Routine2() {
  //   //return Commands.sequence(new GrabberOpen(), new Odometry(new Pose2d(2.161, 6.226, Rotation2d.fromDegrees(90))).raceWith(new WaitCommand(3)), new Odometry(new Pose2d(1.159, 6.21, Rotation2d.fromDegrees(90))).alongWith(new armMove(positionStates.PICKUP)).raceWith(new WaitCommand(4)), new GrabberClose(), new WaitCommand(1), new armMove(positionStates.REST));
  // }

  private Testing() {

    throw new UnsupportedOperationException("This is a utility class!");
  }
}
