// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.ArrayList;
// import java.util.List;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;
// import frc.robot.Constants.ArmLift;
// import frc.robot.commands.ArmGrab.GrabberClose;
// import frc.robot.commands.ArmLift.armMove;
// import frc.robot.commands.Drivetrain.Odometry;
// import frc.robot.subsystems.ArmGrab;
// import frc.robot.subsystems.ArmGrab.States;
// import frc.robot.subsystems.ArmLift.positionStates;


// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class AutomationMiddle extends SequentialCommandGroup {
//   /** Creates a new automations. */
//   public AutomationMiddle() {
//     List<Pose2d> poses = new ArrayList<>();
//     poses.
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     if (DriverStation.getAlliance().equals("Red")) {
//       if (RobotContainer.m_grabber.getArmGrabState() != States.CLOSED) {
//         addCommands(new Odometry(new Pose2d(new Translation2d(Constants.Position2d.RED_TEAM_PICKUP_X_VALUE, Constants.Position2d.RED_TEAM_PICKUP_Y_VALUE), new Rotation2d(0))), new armMove(positionStates.FLOOR), new GrabberClose());
//       }
//       else {
//         if(RobotContainer.coneMode.getAsBoolean()) {
//             new Pose2d().nearest(new Pose2d(), new Pose2d())
//         //   addCommands(new Odometry(new Pose2d().nearest(new Pose2d(Constants.Position2d.SECOND_RED_TEAM_CONE_X_VALUE, Constants.Position2d.SECOND_RED_TEAM_CONE_Y_VALUE, new Rotation2d(0)), 
//         //   new Pose2d(Constants.Position2d.THIRD_RED_TEAM_CONE_X_VALUE, Constants.Position2d.THIRD_RED_TEAM_CONE_Y_VALUE, new Rotation2d(0)),
//         //   new Pose2d(Constants.Position2d.FOURTH_RED_TEAM_CONE_X_VALUE, Constants.Position2d.FOURTH_RED_TEAM_CONE_Y_VALUE, new Rotation2d(0)), 
//         //   new Pose2d(Constants.Position2d.FIFTH_RED_TEAM_CONE_X_VALUE, Constants.Position2d.FIFTH_RED_TEAM_CONE_Y_VALUE, new Rotation2d(0)), 
//         //   new Pose2d(Constants.Position2d.SIXTH_RED_TEAM_CONE_X_VALUE, Constants.Position2d.SIXTH_RED_TEAM_CONE_Y_VALUE, new Rotation2d(0)))));
//         }
//         else {
//           addCommands(new Odometry(new Pose2d(new Translation2d(Constants.Position2d.FIRST_RED_TEAM_CUBE_X_VALUE, Constants.Position2d.FIRST_RED_TEAM_CUBE_Y_VALUE), new Rotation2d(0))), new armMove(positionStates.CUBE), new GrabberClose());
//         }
//       }
    
//     }

   
//   }

// }