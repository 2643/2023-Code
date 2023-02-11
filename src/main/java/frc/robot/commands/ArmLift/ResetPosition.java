package frc.robot.commands.ArmLift;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ResetPosition extends CommandBase {
  
  /** Creates a new ResetPosition. */
  double resetPosition;
  boolean reset = false;
  double increment;
  int state = 1;
  
  public ResetPosition() {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_armLift);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_armLift.reset();
    RobotContainer.m_armLift.changeVelocity(7000);
    RobotContainer.m_armLift.movePos(2048*100*4.5);

  }

  @Override
  public void execute() {
    
    increment += 1;
    if (state == 1) {
      System.out.println("state 1");
      if (RobotContainer.m_armLift.getLimitSwitch()) {
          state = 2;
          increment = 0;
          resetPosition = RobotContainer.m_armLift.getPos();
          RobotContainer.m_armLift.movePos(resetPosition-10000);
          RobotContainer.m_armLift.starttimer();
          System.out.println("state2");
      }
    }

    if (state == 2) {
      System.out.println(RobotContainer.m_armLift.gettimer());
        if(RobotContainer.m_armLift.gettimer()>3){
          System.out.println("state3");
          RobotContainer.m_armLift.changeVelocity(1000);
          RobotContainer.m_armLift.movePos(resetPosition+5000);
          state=3;
        }
           
    }
    // }
    if (state == 3) {
      RobotContainer.m_armLift.stoptimer();
      System.out.println("state 3 still");
      if(RobotContainer.m_armLift.getLimitSwitch()) {
        RobotContainer.m_armLift.reset();
        System.out.println("state 4");
        state = 4;
      }
    }
  }
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_armLift.changeVelocity(100000);
    
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (state == 4) {
      return true;
    }
    return false;
  }
}