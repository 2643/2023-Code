package frc.robot.commands.ArmLift;

import java.sql.Time;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ResetPosition extends CommandBase {
  
  /** Creates a new ResetPosition. */
  double resetPosition;
  boolean finish = false;
  public static enum states{
    state1,
    state2,
    state3,
    state4
   }

  states state = states.state1;


  
  public ResetPosition() {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_armLift);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_armLift.reset();
    RobotContainer.m_armLift.changeVelocity(7000);
    // if(!RobotContainer.m_armLift.getLimitSwitchTwo()){
    //   state = states.state4;
    // }
    state = states.state1;
    RobotContainer.m_armLift.movePos(-2048*100);
    
  }

  @Override
  public void execute() {
    switch(state) {
      case state1:
        System.out.println("state 1");
        if (RobotContainer.m_armLift.getLimitSwitch()) {
            state=states.state2;
            resetPosition = RobotContainer.m_armLift.getPos();
            RobotContainer.m_armLift.movePos(resetPosition+5000);
            RobotContainer.m_armLift.starttimer();
            System.out.println("state2");
          } 
          break;
      case state2:
        System.out.println(RobotContainer.m_armLift.gettimer());
        if(RobotContainer.m_armLift.gettimer()>2){
          System.out.println("state3");
          RobotContainer.m_armLift.changeVelocity(1000);
          RobotContainer.m_armLift.movePos(resetPosition-10000);
          state=states.state3;
          break;
        }
        break;
      case state3:
        RobotContainer.m_armLift.stoptimer();
        System.out.println("state 3 still");
        if(RobotContainer.m_armLift.getLimitSwitch()) {
          RobotContainer.m_armLift.reset();
          System.out.println("state 4");
          state = states.state4;
          Constants.armInitialized = true;
          System.out.println("arm has initialized");
          finish = true;
        }
        break;
        default: System.out.print("Rayirth's & Kaushik's code is trash");
      }
    }
      
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_armLift.changeVelocity(100000);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(state==states.state4){
      return true;
    }
    return false;
  }
}