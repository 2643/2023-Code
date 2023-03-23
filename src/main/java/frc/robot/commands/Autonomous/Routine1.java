package frc.robot.commands.Autonomous;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

import java.util.List;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Routine1 extends SequentialCommandGroup {
    public Routine1(Drivetrain s_Swerve) {
    //     PathPlannerTrajectory trajectory1 = PathPlanner.generatePath(new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
    //                       AutoConstants.kMaxAccelerationMetersPerSecondSquared), 
    //   new PathPoint(new Translation2d(0,0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)),
    //   new PathPoint(new Translation2d(-1,0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0))
    // );
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI/2);
        TrajectoryConfig config = new TrajectoryConfig(Constants.AUTONOMOUS_VELOCITY_PER_SECOND, Constants.AUTONOMOUS_VELOCITY_PER_SECOND/2).setKinematics(Drivetrain.m_kinematics);

        // An example trajectory to follow.  All units in metcers.
        //Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(RobotContainer.m_drivetrain.getPose(), List.of(new Translation2d(12, 1), new Translation2d(13, 2), new Translation2d(14, 2.5), new Translation2d(14.5, 2.75)), new Pose2d(new Translation2d(14.667, 3), new Rotation2d(0)), config);
        Trajectory trajectoryOne = TrajectoryGenerator.generateTrajectory(RobotContainer.m_drivetrain.getPose(), List.of(new Translation2d(12, 1)), new Pose2d(new Translation2d(14, 3), new Rotation2d(0)), config);

        ProfiledPIDController thetaController = new ProfiledPIDController(0.001, 0, 0, constraints);
        thetaController.enableContinuousInput(0, 2*Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectoryOne, s_Swerve::getPose, Drivetrain.m_kinematics, new PIDController(0.001, 0, 0), new PIDController(0.001, 0, 0), thetaController, s_Swerve::setModuleStates, s_Swerve);
        

        addCommands(swerveControllerCommand);
    }

    public Routine1(Pose2d autonomousEndingLocationOne) {
    }
}