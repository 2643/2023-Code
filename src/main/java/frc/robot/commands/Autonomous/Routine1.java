package frc.robot.commands.Autonomous;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Drivetrain.test;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Routine1 extends SequentialCommandGroup {
    public Routine1(Drivetrain s_Swerve) {
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.AUTONOMOUS_RADIANS_PER_SECOND, Constants.AUTONOMOUS_RADIANS_PER_SECOND/2);
        TrajectoryConfig config = new TrajectoryConfig(Constants.AUTONOMOUS_VELOCITY_PER_SECOND, Constants.AUTONOMOUS_VELOCITY_PER_SECOND/2).setKinematics(Drivetrain.m_kinematics);

        // An example trajectory to follow.  All units in meters.
        //Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(RobotContainer.m_drivetrain.getPose(), List.of(new Translation2d(12, 1), new Translation2d(13, 2), new Translation2d(14, 2.5), new Translation2d(14.5, 2.75)), new Pose2d(new Translation2d(14.667, 3), new Rotation2d(0)), config);
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(RobotContainer.m_drivetrain.getPose(), List.of(new Translation2d(12, 1)), new Pose2d(new Translation2d(14.667, 3), new Rotation2d(0)), config);

        var thetaController = new ProfiledPIDController(0.1, 0, 0, constraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory, s_Swerve::getPose, Drivetrain.m_kinematics, new PIDController(0.2, 0, 0), new PIDController(0.2, 0.0001, 0), thetaController, s_Swerve::setModuleStates, s_Swerve);


        addCommands(new InstantCommand(() -> new test()), swerveControllerCommand);
    }
}