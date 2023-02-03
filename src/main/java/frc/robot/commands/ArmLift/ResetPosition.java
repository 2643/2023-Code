package frc.robot.commands.ArmLift;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
public class ResetPosition extends CommandBase {
  /** Creates a new ResetPosition. */
  public ResetPosition() {
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_armLift);
    
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_armLift.movePos(2048*100*4.5);


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_armLift.setPos(0);
    RobotContainer.m_armLift.movePos(0);
  }
    
    

 

    // RobotContainer.m_armLift.setPos(0);
    // RobotContainer.m_armLift.changeAcceleration(0);
    // Timer.delay(1);
    // RobotContainer.m_armLift.movePos(0);
    // RobotContainer.m_armLift.changeAcceleration(10000);
   
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  if(Robotcontainer.m_armLift.getLimitSwitch()){
    return true;
  }

    return false;
  }
}