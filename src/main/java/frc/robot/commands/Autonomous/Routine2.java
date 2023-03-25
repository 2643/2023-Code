// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Drivetrain.Odometry;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Routine2 extends SequentialCommandGroup {
  /** Creates a new Routine2. */
  public Routine2() {
    
    // addCommands(new Odometry(new Pose2d(new Translation2d(14.607, 2.739), new Rotation2d(0))), 
    // new Odometry(new Pose2d(new Translation2d(14.937, 2.739), new Rotation2d(180))), 
    // new Odometry(new Pose2d(new Translation2d(15.323, 2.739), new Rotation2d(0))),
    // new Odometry(new Pose2d(new Translation2d(14.766, 2.739), new Rotation2d(-55))),
    // new Odometry(new Pose2d(new Translation2d(12.855, 5.423), new Rotation2d(165))),
    // new Odometry(new Pose2d(new Translation2d(14.766, 3.655), new Rotation2d(140))),
    // new Odometry(new Pose2d(new Translation2d(12.924, 3.655), new Rotation2d(0))), 
    // new armMove(positionStates.CUBE), new GrabberOpen(), new Odometry(Constants.Position2d.ORIGINAL_BALL_POSITIONING_ONE));
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  } 
}