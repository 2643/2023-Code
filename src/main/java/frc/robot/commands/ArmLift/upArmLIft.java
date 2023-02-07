// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.ArmLift;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.RobotContainer;

// public class upArmLIft extends CommandBase {
//   /** Creates a new upArmLIft. */
//   public upArmLIft() {
//     addRequirements(RobotContainer.m_armLift);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
    
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     RobotContainer.m_armLift.changeVelocity(100000);
//     RobotContainer.m_armLift.afterRestMovePos(0);

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     if(RobotContainer.m_armLift.getPos()==0){
//       return true;
//     }
//     return false;
//   }
// }
