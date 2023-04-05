package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import java.time.Duration;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class ArmGrab extends SubsystemBase {
  static boolean armGrabInitialized = false;
  Timer timer = new Timer();
  DigitalInput limitSwitchInput = new DigitalInput(Constants.ArmGrab.GRABBER_LIMIT_SWITCH_PORT);
  TalonFX WinchMotor = new TalonFX(Constants.ArmGrab.GRABBER_MOTOR_PORT);
   
  double kP = 0;
  double posKP = 0.2;
  double kF = 0.045;
  double kI;
  double kD;
  double rpm;
  double currentVel;
  double currentPos;
  double currentCURRENT = 0.0;
  double targetRPM = 1000;

  // GenericEntry pEntry = Shuffleboard.getTab("PID but better").add("Proportional", kP).getEntry();
  // GenericEntry iEntry = Shuffleboard.getTab("PID but better").add("Integral", kI).getEntry();
  // GenericEntry dEntry = Shuffleboard.getTab("PID but better").add("Derivative", kD).getEntry();
  // GenericEntry fEntry = Shuffleboard.getTab("PID but better").add("Feed Forward", kF).getEntry();
  ShuffleboardLayout armGrabLayout = Shuffleboard.getTab("Debug").getLayout("ArmGrab", BuiltInLayouts.kGrid).withSize(4, 2).withPosition(4, 3);
  GenericEntry targetVelEntry = armGrabLayout.add("Target", targetRPM).withPosition(0, 0).getEntry();
  GenericEntry currentVelEntry = armGrabLayout.add("Current velocity", currentVel).withPosition(1, 0).getEntry();
  GenericEntry currentCURRENTEntry = armGrabLayout.add("Current CURRENT", currentCURRENT).withPosition(0, 1).getEntry();
  GenericEntry stateEntry = armGrabLayout.add("Current state", "NOT_INITIALIZED").withPosition(1, 1).getEntry();

  public ArmGrab() {
    WinchMotor.configFactoryDefault();
    WinchMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    WinchMotor.setInverted(true);
    WinchMotor.setNeutralMode(NeutralMode.Brake);
    WinchMotor.config_kF(0, kF);
    WinchMotor.config_kP(0, kP);
    WinchMotor.config_kI(0, kI);
    WinchMotor.config_kD(0, kD);
    WinchMotor.config_kP(1, posKP);
    resetPosition();
  }

  public enum States{
    NOT_INITIALIZED,
    INITIALIZING_OPENING,
    INITIALIZING_STOPPING,
    INITIALIZED,
    OPENING,
    OPENED,
    CLOSING_STARTING_VELOCITY,
    CLOSING_TIME_ELAPSING,
    CLOSING_CURRENT,
    CLOSED;
  }

  

  static States state = States.CLOSING_STARTING_VELOCITY;

  public void setRPM(double rpm) {
    WinchMotor.selectProfileSlot(0, 0);
    WinchMotor.set(ControlMode.Velocity, rpm);
}

// public void setPercentOutput(double percent) {
//     WinchMotor.set(ControlMode.PercentOutput, percent);
// }

public void stopMotor() {
    WinchMotor.set(ControlMode.PercentOutput, 0);
}

public double getCurrentOutput() {
    return WinchMotor.getStatorCurrent();
}

public void resetPosition() {
    WinchMotor.setSelectedSensorPosition(0);
}

public double getCurrentPosition() {
    return WinchMotor.getSelectedSensorPosition();
}

public double getMotorVelocity() {
    return WinchMotor.getSelectedSensorVelocity();
}

public void setArmGrabState(States state) {
    ArmGrab.state = state;
}

public States getArmGrabState() {
    return state;
}

public boolean getArmGrabInitialized() {
    return armGrabInitialized;
}

public void firstCurrentPass() {
    timer.start();
    if (timer.hasElapsed(0.2)) {
        timer.stop();
        timer.reset();
        state = States.CLOSING_CURRENT;
        timer.start();
    }
}

public void firstCurrentPassCone() {
    timer.start();
    
    if (timer.hasElapsed(0.2)) {
        timer.stop();
        timer.reset();
        state = States.CLOSING_CURRENT;
    }
}

public void movePos(double pos) {
    WinchMotor.selectProfileSlot(1, 0);
    WinchMotor.set(ControlMode.Position, pos);
}

public boolean isClosed() {
    return state == States.CLOSED || state == States.CLOSING_CURRENT || state == States.CLOSING_STARTING_VELOCITY || state == States.CLOSING_TIME_ELAPSING;
}

  @Override
  public void periodic(){
    targetVelEntry.setDouble(targetRPM);
    //System.out.println("Current: " + getCurrentOutput() + " Velocity: " + getMotorVelocity() + " Limit Switch: " + limitSwitchInput.get());
    currentCURRENT = getCurrentOutput();
    currentCURRENTEntry.setDouble(currentCURRENT);
    currentVel = WinchMotor.getSelectedSensorVelocity();
    currentVelEntry.setDouble(currentVel);

    if(DriverStation.isEnabled()) {
      switch(state) {
        case NOT_INITIALIZED:
          stateEntry.setString("Not Initialized");
          state = States.INITIALIZING_OPENING;
          break;
        case INITIALIZING_OPENING:
          targetRPM = -Constants.ArmGrab.GRABBER_TARGET_RPM;
          setRPM(targetRPM*0.7);
          if(limitSwitchInput.get()){
            state = States.INITIALIZING_STOPPING;
          }

          stateEntry.setString("Initializing-- Opening");
          break;
        case INITIALIZING_STOPPING:
          targetRPM = 0;
          setRPM(targetRPM);
          resetPosition();
          state = States.INITIALIZED;
          stateEntry.setString("Initializing-- Stopping");
          break;
        case INITIALIZED:
          armGrabInitialized = true;
          stateEntry.setString("Initialized");
          state = States.OPENING;
          break;
        case OPENING:
        stateEntry.setString("Opening");
          if(!armGrabInitialized) {
            ArmGrab.state = States.NOT_INITIALIZED;
            break;
          }
          targetRPM = -Constants.ArmGrab.GRABBER_TARGET_RPM;
          setRPM(targetRPM);
          if(getCurrentPosition() <= Constants.ArmGrab.GRABBER_MAX_OPEN_POS) {
            targetRPM = 0;
            setRPM(targetRPM);
            state = States.OPENED;
          }
          break;
        case OPENED:
          stateEntry.setString("Opened");
          break;
        case CLOSING_STARTING_VELOCITY:
          stateEntry.setString("Waiting for Velocity spike");
          setRPM(Constants.ArmGrab.GRABBER_TARGET_RPM);
          if(currentVel >= targetRPM * 0.8) {
            state = States.CLOSING_TIME_ELAPSING;
          }
          break;
        case CLOSING_TIME_ELAPSING:
          stateEntry.setString("Waiting for time lapse");
          if(RobotContainer.coneMode.getAsBoolean()) {
            firstCurrentPassCone();
            // timer.reset();
            // timer.start();
          } else {
            firstCurrentPass();
          }
          break;
        case CLOSING_CURRENT: 
          stateEntry.setString("Closing current");
          if(RobotContainer.coneMode.getAsBoolean()) {
            if(getCurrentOutput() >= Constants.ArmGrab.TARGET_CONE_CURRENT_VALUE) {
              // if(timer.hasElapsed(0.3))
              movePos(getCurrentPosition());  
              ArmGrab.state = States.CLOSED; 
            }
          } else {
            if(getCurrentOutput() >= Constants.ArmGrab.TARGET_CUBE_CURRENT_VALUE) {
              movePos(getCurrentPosition());
              ArmGrab.state = States.CLOSED;
            }
          }
          break;
        case CLOSED:
          
          currentCURRENT = getCurrentOutput();
          currentCURRENTEntry.setDouble(currentCURRENT);
          targetVelEntry.setDouble(targetRPM);
          stateEntry.setString("Closed");
          break;
        default:
          stateEntry.setString("Why are you in default");
          break;
      }
    }
  }

}
