// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;

public class ElonMusk extends CommandBase {
  /** Creates a new ElonMusk. */
  public ElonMusk() {
    addRequirements(RobotContainer.m_grabber);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  Timer timer = new Timer();
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.hasElapsed(3)){
      RobotContainer.m_grabber.firstCurrentPass();
      timer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
