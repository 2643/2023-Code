// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import frc.robot.commands.Drivetrain.Odometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto() {
    return Commands.sequence(new Odometry(new Pose2d(new Translation2d(0, -17.16667), new Rotation2d(0))), new WaitCommand(1), new Odometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0))), new WaitCommand(1), new Odometry(new Pose2d(new Translation2d(-17.16667, 0), new Rotation2d(0))));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
