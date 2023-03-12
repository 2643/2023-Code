// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmLift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.ArmLift.moveArmJoystick;

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
    if (!Constants.armCalled) {
      if (moveDirection == ArmLift.moveArmJoystick.Up) {
        targetPos = RobotContainer.m_armLift.getPos() + 2000;
      }
      // } else if (moveDirection == ArmLift.moveArmJoystick.Encoder) {
      //   targetPos = controlToMultiplier(encoderInput) * 100 * 4.5 * 5.69;
      //   RobotContainer.m_armLift.movePosFF(targetPos);
      else if (moveDirection == ArmLift.moveArmJoystick.Down) {
        targetPos = RobotContainer.m_armLift.getPos() - 2000;
      }
  }
  }
  // private int controlToMultiplier(double ctrlValue) {
  //   if (ctrlValue > 0.6) {
  //     return 0;
  //   } else if (ctrlValue > 0.4) {
  //     return -29;
  //   } else if (ctrlValue > 0.2) {
  //     return -58;
  //   } else if (ctrlValue > 0){
  //       return -87;
  //   } else if (ctrlValue > -0.5){
  //       return -116;
  //   } else {
  //       return -145;
  //     }
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}