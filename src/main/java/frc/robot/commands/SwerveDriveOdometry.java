// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveDriveOdometry extends CommandBase {
  Pose2d pos;
  double targetXPos;
  double targetYPos;
  DoubleSupplier x_vel;
  DoubleSupplier y_vel;
  DoubleSupplier turn_vel;
  double targetTurnDegrees;
  /** Creates a new SwerveDriveOdometry. */
  public SwerveDriveOdometry(Pose2d m_targetPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerve);
    
    targetXPos = m_targetPos.getX();
    targetYPos = m_targetPos.getY();
    targetTurnDegrees = m_targetPos.getRotation().getDegrees();
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pos = RobotContainer.m_swerve.m_odometry.getPoseMeters();
    //System.out.println("called");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    pos = RobotContainer.m_swerve.m_odometry.getPoseMeters();

    if(Math.round(pos.getX()*10) != Math.round(targetXPos*10)){
      if(pos.getX() < targetXPos) {
        x_vel = () -> Constants.AUTONOMOUS_VELOCITY_PER_SECOND * 0.2;
      } else if(pos.getX() > targetXPos) {
        x_vel = () -> -Constants.AUTONOMOUS_VELOCITY_PER_SECOND * 0.2;
      }
    } else {
      x_vel = () -> 0;
    }

    if(Math.round(pos.getY()*10) != Math.round(targetYPos*10)){
      if(pos.getY() < targetYPos) {
        y_vel = () -> Constants.AUTONOMOUS_VELOCITY_PER_SECOND * 0.2;
      } else if(pos.getY() > targetYPos) {
        y_vel = () -> -Constants.AUTONOMOUS_VELOCITY_PER_SECOND * 0.2;
      }
    } else {
      y_vel = () -> 0;
    }

    // if(Math.round(RobotContainer.m_swerve.gyroAngle().getDegrees()) != Math.round(targetTurnDegrees)){
    //   if(pos.getRotation().getDegrees() < targetTurnDegrees) {
    //     turn_vel = Constants.AUTONOMOUS_RADIANS_PER_SECOND * 0.1;
    //   } else if(pos.getRotation().getDegrees() > targetTurnDegrees) {
    //     turn_vel = -Constants.AUTONOMOUS_RADIANS_PER_SECOND * 0.1;
    //   }
    // } else {
    //  turn_vel = 0;
    // }

    //CHANGE
    if(Math.round(targetTurnDegrees / 5) == Math.round(RobotContainer.m_swerve.gyroAngle().getDegrees() / 5))
      turn_vel = () -> 0;
    else
      turn_vel = () -> 0.2 * Constants.MAX_RADIANS_PER_SECOND * Math.copySign(1, -targetTurnDegrees + RobotContainer.m_swerve.gyroAngle().getDegrees()) * -1;



    RobotContainer.m_swerve.setChasisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(x_vel.getAsDouble(), y_vel.getAsDouble(),
        0, RobotContainer.m_swerve.gyroAngle()));
    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_swerve.setChasisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
        0, RobotContainer.m_swerve.gyroAngle()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((Math.round(pos.getX()*10) == Math.round(targetXPos*10)) && ((Math.round(pos.getY()*10) == Math.round(targetYPos*10))) && (Math.round(pos.getRotation().getDegrees() / 5) == (Math.round(targetTurnDegrees / 5)))) {
      //System.out.println("finished");
      return true;
    }
    return false;
  }
}
