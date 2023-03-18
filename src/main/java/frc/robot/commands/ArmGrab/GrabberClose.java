// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmGrab;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmGrab.States;
import edu.wpi.first.wpilibj.Timer;

public class GrabberClose extends CommandBase {
  public GrabberClose() {
    addRequirements(RobotContainer.m_grabber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  Timer timer = new Timer();
  //boolean firstCurrent = true;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
    if(RobotContainer.m_grabber.getArmGrabState() != States.CLOSED) {
      RobotContainer.m_grabber.setArmGrabState(States.CLOSING_STARTING_VELOCITY);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
