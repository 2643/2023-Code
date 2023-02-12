// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmLift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmLift;
//mport frc.robot.subsystems.ArmLift.moveArmJoystick;

public class MoveArm extends CommandBase {
  ArmLift.moveArmJoystick moveDirection;
  static double targetPos;
  
  //double targetPos;
  /** Creates a new ArmLift. */
  public MoveArm(ArmLift.moveArmJoystick moveDirection) {
    addRequirements(RobotContainer.m_armLift);
    this.moveDirection = moveDirection;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Working");
    if(moveDirection == ArmLift.moveArmJoystick.Up) {
      targetPos = RobotContainer.m_armLift.getPos() + 10000;
      RobotContainer.m_armLift.movePos(targetPos);
    } else {
      targetPos = RobotContainer.m_armLift.getPos() - 10000;
      RobotContainer.m_armLift.movePos(targetPos);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.m_armLift.getPos() == 0){
      return true;
    }
    return false;
  }
}
