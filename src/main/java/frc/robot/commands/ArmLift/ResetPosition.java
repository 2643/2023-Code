package frc.robot.commands.ArmLift;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
public class ResetPosition extends CommandBase {
  /** Creates a new ResetPosition. */
  double resetPosition;
  boolean reset = false;
  double increment;
  int state=1;
  public ResetPosition() {
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_armLift);
    
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_armLift.setPos(0);
    RobotContainer.m_armLift.movePos(0);
    

 
    
    
    
    //   if(RobotContainer.m_armLift.getLimitSwitch()){
    //     RobotContainer.m_armLift.movePos(resetPosition);
    //     reset=true;
    //   }
    }
    // RobotContainer.m_armLift.setPos(0);
    // RobotContainer.m_armLift.movePos(0);
 

    //RobotContainer.m_armLift.changeVelocity(100000);
    //RobotContainer.m_armLift.setPos(0);
    
    // if(RobotContainer.m_armLift.getLimitSwitch()){
    //   RobotContainer.m_armLift.setPos(0);
    // }
    
  
  // Called every time the scheduler runs while the command is scheduled.
  
  @Override
  public void execute() {
    increment+=1;
    if(state==1){
      RobotContainer.m_armLift.movePos(increment*1000);
      System.out.println("state 1");
      if(RobotContainer.m_armLift.getLimitSwitch()){
        state=2;
      }
      
    }
    if(RobotContainer.m_armLift.getLimitSwitch()&&state==2){
      increment=0;
      resetPosition=RobotContainer.m_armLift.getPos();
      RobotContainer.m_armLift.movePos(resetPosition-20000);
      System.out.println("state 2");  
      if(!RobotContainer.m_armLift.getLimitSwitch()){
        state=3;
        System.out.println("state 3");
        
      }
    }
    if(state==3){
      increment+=1;
      RobotContainer.m_armLift.movePos(increment*1000);
      System.out.println("state 3");
      state=4;
    }
    // if(RobotContainer.m_armLift.getPos()==resetPosition-10000){
    //   System.out.println("inside");
    //   RobotContainer.m_armLift.movePos(resetPosition);
    //   if(RobotContainer.m_armLift.getLimitSwitch()){
    //     reset=true;
    //   }
    
      // if(RobotContainer.m_armLift.getLimitSwitch()){
      //   reset=true;
      // }
      //RobotContainer.m_armLift.movePos(resetPosition);
      
     


  
  
  }

    //RobotContainer.m_armLift.speedControl(0.6);
    //
    // if(RobotContainer.m_armLift.getLimitSwitch())
    //   RobotContainer.m_armLift.setPos(0);
      
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.m_armLift.changeVelocity(100000);
    //RobotContainer.m_armLift.speedControl(0);
    //RobotContainer.m_armLift.movePos(resetPosition);
    RobotContainer.m_armLift.setPos(0);
    RobotContainer.m_armLift.movePos(0);
    
    
    //RobotContainer.m_armLift.movePos(RobotContainer.m_armLift.getPos());
    

    
  }
    
    

 

    // RobotContainer.m_armLift.setPos(0);
    // RobotContainer.m_armLift.changeAcceleration(0);
    // Timer.delay(1);
    // RobotContainer.m_armLift.movePos(0);
    // RobotContainer.m_armLift.changeAcceleration(10000);
   
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(state==4){
      return true;
    }
    return false;
  }
}