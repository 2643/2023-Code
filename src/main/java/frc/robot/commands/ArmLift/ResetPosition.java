// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmLift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ResetPosition extends CommandBase {
  /** Creates a new ResetPosition. */
  public ResetPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_armLift);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_armLift.movePos(2048*100);
    if (RobotContainer.m_armLift.getLimitSwitch()) {
      // We are going up and top limit is tripped so stop
      RobotContainer.m_armLift.setPos(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.m_armLift.getLimitSwitch()){
      return true;
    }
    return false;
  }
}
