package frc.robot.commands.ArmLift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.ArmLift.ArmLiftStates;

public class ResetPosition extends CommandBase {
  //public int target;
  public double resetPosition;
  
  /** Creates a new ResetPosition. */
  boolean finish = false;
  // public static enum states{
  //   state1,
  //   state2,
  //   state3,
  //   state4
  //  }

  // states state = states.state1;


  
  public ResetPosition() {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_armLift);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmLift.ArmLiftState = ArmLiftStates.INITIALIZING;
    RobotContainer.m_armLift.reset();
    RobotContainer.m_armLift.changeVelocity(1000);//4000
    RobotContainer.m_armLift.movePos(Constants.COUNT_PER_DEGREES * 75);
    System.out.println("state1");
    // if(!RobotContainer.m_armLift.getLimitSwitchTwo()){
    //   state = states.state4;
    // }
    // state = states.state1;
    // RobotContainer.m_armLift.movePos(-2048*100);
    
  }


  @Override
  public void execute() {
      if(RobotContainer.m_armLift.getPos() >= Constants.COUNT_PER_DEGREES * 70) {
        System.out.println("state2");
        RobotContainer.m_armLift.changeVelocity(500);//1000
        RobotContainer.m_armLift.movePos(Constants.COUNT_PER_DEGREES * 75);
        
      }
      
      if (RobotContainer.m_armLift.getLimitSwitch()) {
        resetPosition = RobotContainer.m_armLift.getPos();
        System.out.println("state3");
        finish = true;
        ArmLift.ArmLiftState = ArmLiftStates.INITIALIZED;
        RobotContainer.m_armLift.movePos(resetPosition);
      }
  }
      
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_armLift.changeVelocity(8533);//10000
    RobotContainer.m_armLift.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}