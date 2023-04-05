// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmLift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmLift.*;
//import frc.robot.subsystems.ArmLift.ArmLiftStates;

public class MoveArm extends CommandBase {
  moveArmJoystick moveDirection;
  public static double targetPos;
  double encoderInput;
  boolean finish = false;

  //double targetPos;
  /** Creates a new ArmLift. */
  public MoveArm(moveArmJoystick moveDirection) {
    addRequirements(RobotContainer.m_armLift);
    this.moveDirection = moveDirection;
  }
  



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    encoderInput = RobotContainer.operatorBoard.getRawAxis(1);
  
    if (RobotContainer.m_armLift.getArmLiftState() == ArmLiftStates.INITIALIZED) {
      if (moveDirection == moveArmJoystick.Up) {
        if(targetPos > Constants.ArmLift.TOP_SOFT_LIMIT_MOVEPOS)
          targetPos = RobotContainer.m_armLift.getPos() + 2000;
      } else if (moveDirection == moveArmJoystick.Encoder) {
          targetPos = RobotContainer.m_armLift.getCurrentEncoderPos();
      } else if (moveDirection == moveArmJoystick.Down) {
        if(targetPos < Constants.ArmLift.BOTTOM_SOFT_LIMIT_MOVEPOS)
          targetPos = RobotContainer.m_armLift.getPos() - 2000;
      }
    }
  // } else if(RobotContainer.m_armLift.getArmLiftState() == ArmLiftStates.NOT_INITIALIZED){
  //     if (moveDirection == moveArmJoystick.Up) {
  //       if(targetPos > Constants.ArmLift.TOP_SOFT_LIMIT_MOVEPOS)
  //         targetPos = RobotContainer.m_armLift.getPos() + 500;
  //     } else if (moveDirection == moveArmJoystick.Down) {
  //       if(targetPos < Constants.ArmLift.BOTTOM_SOFT_LIMIT_MOVEPOS)
  //         targetPos = RobotContainer.m_armLift.getPos() - 500;
  //     }
  //   }
    finish = true;
  }
    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}