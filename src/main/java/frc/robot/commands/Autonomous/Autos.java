// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Drivetrain.Odometry;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase Routine1() {
    //return Commands.sequence(new Odometry(new Pose2d(new Translation2d(14.664, 0.5), new Rotation2d(0))), new Odometry(new Pose2d(new Translation2d(14.664, 2.159), new Rotation2d(0))));
    return Commands.sequence(new Odometry(new Pose2d(new Translation2d(2.483, 7.991), new Rotation2d(0))));
    //return Commands.sequence( new WaitCommand(0));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
