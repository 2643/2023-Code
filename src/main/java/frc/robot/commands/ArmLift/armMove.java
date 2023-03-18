// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmLift;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmLift;

public class armMove extends CommandBase {
  ArmLift.positionStates state;

  /** Creates a new armMove. */
  public armMove(ArmLift.positionStates state) {
    addRequirements(RobotContainer.m_armLift);
    this.state=state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if(state == ArmLift.positionStates.REST){
      RobotContainer.m_armLift.movePosFF(Constants.ArmLift.REST);
    }
    else if(state == ArmLift.positionStates.FLOOR){
      RobotContainer.m_armLift.movePosFF(Constants.ArmLift.FLOOR);
    }
    else if(state == ArmLift.positionStates.CUBE){
      RobotContainer.m_armLift.movePosFF(Constants.ArmLift.CUBE);
    }
    else if(state == ArmLift.positionStates.CONE){
      RobotContainer.m_armLift.movePosFF(Constants.ArmLift.CONE);
    }
    else if(state == ArmLift.positionStates.PICKUP){
      RobotContainer.m_armLift.movePosFF(Constants.ArmLift.PICKUP);
    }
    else if(state == ArmLift.positionStates.CHARGING_STATION){
      RobotContainer.m_armLift.movePosFF(Constants.ArmLift.CHARGING_STATION);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}