// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automation;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  boolean firstOffset = false;
  boolean finished = false;
  Timer time = new Timer();
  public AutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    firstOffset = false;
    finished = false;
    time.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(RobotContainer.m_drivetrain.roll().getDegrees()) > 3 && !firstOffset) {
      firstOffset = true;
    }

    if(firstOffset) {
      if(Math.abs(RobotContainer.m_drivetrain.roll().getDegrees()) < 5) {
        time.start();
        RobotContainer.m_drivetrain.setChassisSpeed(new ChassisSpeeds(0, 0, 0));
        if(time.get() > 1) {
          finished = true;
        }
      } else if(RobotContainer.m_drivetrain.roll().getDegrees() > 0) {
        time.reset();
        RobotContainer.m_drivetrain.setChassisSpeed(new ChassisSpeeds(0, -0.12*Constants.MAX_METERS_PER_SECOND, 0));
      } else {
        time.reset();
        RobotContainer.m_drivetrain.setChassisSpeed(new ChassisSpeeds(0, 0.12*Constants.MAX_METERS_PER_SECOND, 0));
      }
    } else {
      RobotContainer.m_drivetrain.setChassisSpeed(new ChassisSpeeds(0, -0.5*Constants.MAX_METERS_PER_SECOND, 0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
