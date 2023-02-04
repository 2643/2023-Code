// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BillGates;

public class steveJobs extends CommandBase {
  /** Creates a new steveJobs. */
  public steveJobs() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_grabber);
  }

  boolean HitCurrent = false;

  public void finish(){
    RobotContainer.m_grabber.setRPM(0);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //RobotContainer.m_grabber.firstCurrentPass();
    if(!HitCurrent){
      if(RobotContainer.m_grabber.getCurrentOutput() >= 3.6 && RobotContainer.m_grabber.getCurrentOutput() <= 4){
        finish();  
        HitCurrent = true;
      }
      System.out.println(HitCurrent);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_grabber.setRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return HitCurrent;
  }
}