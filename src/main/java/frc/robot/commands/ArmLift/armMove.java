// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmLift;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.ArmLift.positionStates;

public class armMove extends CommandBase {
  ArmLift.positionStates state;

  /** Creates a new armMove. */
  public armMove(ArmLift.positionStates state) {
    addRequirements(RobotContainer.m_armLift);
    this.state=state;
  }

  public double stateToPosition(positionStates s) {
    if(s == ArmLift.positionStates.REST){
      return Constants.ArmLift.REST;
    }
    else if(s == ArmLift.positionStates.FLOOR){
      if(RobotContainer.coneMode.getAsBoolean()){
        return Constants.ArmLift.CONE_FLOOR;
      }
      else{
        return Constants.ArmLift.FLOOR;
      }
    }
    else if(s == ArmLift.positionStates.CUBE){
      return Constants.ArmLift.CUBE;
    }
    else if(s == ArmLift.positionStates.CONE){
      return Constants.ArmLift.CONE;
    }
    else if(s == ArmLift.positionStates.PICKUP){
      return Constants.ArmLift.PICKUP;
    }
    else if(s == ArmLift.positionStates.CHARGING_STATION){
      return Constants.ArmLift.CHARGING_STATION;
    }
    return 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    MoveArm.targetPos = stateToPosition(state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(MoveArm.targetPos == stateToPosition(state) && Math.abs(stateToPosition(state) - RobotContainer.m_armLift.getPos()) < 2000)
      return true;
    return false;
  }
}
