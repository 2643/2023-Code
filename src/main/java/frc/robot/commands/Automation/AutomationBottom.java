package frc.robot.commands.Automation;


import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmGrab.grabberPull;
import frc.robot.commands.ArmLift.armMove;
import frc.robot.commands.Drivetrain.Odometry;
import frc.robot.subsystems.ArmLift.positionStates;
import frc.robot.subsystems.Grabber.States;

public class AutomationBottom extends SequentialCommandGroup {
//   /** Creates a new automations. */
  public AutomationBottom() {
    List<Pose2d> poses = new ArrayList<>();
    
    poses.add(new Pose2d(Constants.Position2d.SECOND_RED_TEAM_CONE_X_VALUE, Constants.Position2d.SECOND_RED_TEAM_CONE_Y_VALUE, new Rotation2d(0)));
    poses.add(new Pose2d(Constants.Position2d.FOURTH_RED_TEAM_CONE_X_VALUE, Constants.Position2d.FOURTH_RED_TEAM_CONE_Y_VALUE, new Rotation2d(0)));
    poses.add(new Pose2d(Constants.Position2d.FIFTH_RED_TEAM_CONE_X_VALUE, Constants.Position2d.FIFTH_RED_TEAM_CONE_Y_VALUE, new Rotation2d(0)));
    poses.add(new Pose2d(Constants.Position2d.SIXTH_RED_TEAM_CONE_X_VALUE, Constants.Position2d.SIXTH_RED_TEAM_CONE_Y_VALUE, new Rotation2d(0)));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if(DriverStation.getAlliance().equals("Red")) {
      if (RobotContainer.m_grabber.getArmGrabState() != States.STOP_MOTOR_PULL) {
        addCommands(new Odometry(new Pose2d(new Translation2d(Constants.Position2d.RED_TEAM_PICKUP_X_VALUE, Constants.Position2d.RED_TEAM_PICKUP_Y_VALUE), new Rotation2d(0))), new armMove(positionStates.FLOOR), new grabberPull());
      }
      else {
        if(RobotContainer.coneMode.getAsBoolean()) {
          addCommands(new Odometry(new Pose2d().nearest(poses)));
        }
        else {
          addCommands(new Odometry(new Pose2d(new Translation2d(Constants.Position2d.FIRST_RED_TEAM_CUBE_X_VALUE, Constants.Position2d.FIRST_RED_TEAM_CUBE_Y_VALUE), new Rotation2d(0))), new armMove(positionStates.CUBE), new grabberPull());
        }
      }
    
    }

   
  }
}


