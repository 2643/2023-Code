// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmLift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.ArmLift.ArmLiftStates;

public class MoveArm extends CommandBase {
  ArmLift.moveArmJoystick moveDirection;
  public static double targetPos;
  double encoderInput;

  //double targetPos;
  /** Creates a new ArmLift. */
  public MoveArm(ArmLift.moveArmJoystick moveDirection) {
    addRequirements(RobotContainer.m_armLift);
    this.moveDirection = moveDirection;
  }
  



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    encoderInput = RobotContainer.m_opBoard.getRawAxis(1);
  
    if (ArmLift.ArmLiftState == ArmLiftStates.INITIALIZED) {
      if (moveDirection == ArmLift.moveArmJoystick.Up) {
        targetPos = RobotContainer.m_armLift.getPos() + 2000;
      } else if (moveDirection == ArmLift.moveArmJoystick.Encoder) {
        targetPos = controlToMultiplier(encoderInput);
      } else if (moveDirection == ArmLift.moveArmJoystick.Down) {
        targetPos = RobotContainer.m_armLift.getPos() - 2000;
      }
  } else {
      if (moveDirection == ArmLift.moveArmJoystick.Up) {
        targetPos = RobotContainer.m_armLift.getPos() + 500;
      } else if (moveDirection == ArmLift.moveArmJoystick.Down) {
        targetPos = RobotContainer.m_armLift.getPos() - 500;
      }
    }
  }

  private double controlToMultiplier(double ctrlValue) {
    if (ctrlValue >= 0.8) {
      return Constants.rest;
    } else if (ctrlValue >= 0.6) {
      return Constants.floor;
    } else if (ctrlValue >= 0.35) {
      return Constants.chargingStation;
    } else if (ctrlValue >= 0.1){
      return Constants.cube;
    } else if (ctrlValue >= 0){
      return Constants.cone;
    } else if(ctrlValue > -0.5){
      return Constants.pickup;
    } else{
      return Constants.rest;
    }
  }
  
// ctrl values 
// floor: 0.73 
// charging station: 0.52 
// cube: 0.29 
// cone: 0.03 
// pickup: -0.35 
// rest: 1 


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}