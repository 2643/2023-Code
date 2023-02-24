// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BillGates;
import frc.robot.subsystems.BillGates.States;

import com.swervedrivespecialties.swervelib.ctre.Falcon500SteerConfiguration;

import edu.wpi.first.wpilibj.Timer;

public class ElonMusk extends CommandBase {
  /** Creates a new ElonMusk. */
  public ElonMusk() {
    addRequirements(RobotContainer.m_grabber);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  Timer timer = new Timer();
  boolean FirstCurrent = true;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    //RobotContainer.m_grabber.WinchMotor.setInverted(true);
    //RobotContainer.m_grabber.
    BillGates.state = States.CLOSINGSTART;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(BillGates.state == States.CLOSINGSTART){
      System.out.println("Bob's a big boy");
      if(RobotContainer.m_grabber.getMotorVelocity()>10){
        RobotContainer.m_grabber.firstCurrentPass();
        System.out.println("Closing Current");
      }
    }
    if(BillGates.state == States.CLOSINGCURRENT){
      System.out.println("pls help");
      System.out.println("Test: " + RobotContainer.m_grabber.getCurrentOutput());
      if(RobotContainer.m_grabber.getCurrentOutput() >= Constants.TARGET_CURRENT_VALUE){
        System.out.println("ReachedLimit");
        RobotContainer.m_grabber.stopMotor();
        BillGates.state = States.CLOSED;
        System.out.println("Closed");
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
    FirstCurrent = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
