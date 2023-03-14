// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ArmGrab extends SubsystemBase {
  /** Creates a new BillGates. */
  public static boolean armGrabInitialized = false;
  Timer timer = new Timer();
  DigitalInput limitSwitchInput = new DigitalInput(Constants.GRABBER_LIMIT_SWITCH_PORT);
  CANSparkMax WinchMotor = new CANSparkMax(Constants.GRABBER_MOTOR_PORT, MotorType.kBrushless);
   
  double kP = 0;
  double kF = 0.0000844;
  double kI;
  double kD;
  double rpm;
  double currentVel;
  double currentPos;
  double currentCURRENT = 0.0;
  double targetRPM = 1000;

  GenericEntry pEntry = Shuffleboard.getTab("PID but better").add("Proportional", kP).getEntry();
  GenericEntry iEntry = Shuffleboard.getTab("PID but better").add("Integral", kI).getEntry();
  GenericEntry dEntry = Shuffleboard.getTab("PID but better").add("Derivative", kD).getEntry();
  GenericEntry fEntry = Shuffleboard.getTab("PID but better").add("Feed Forward", kF).getEntry();
  GenericEntry targetVelEntry = Shuffleboard.getTab("PID but better").add("Target", targetRPM).getEntry();
  GenericEntry currentVelEntry = Shuffleboard.getTab("PID but better").add("Current velocity", currentVel).getEntry();
  GenericEntry currentCURRENTEntry = Shuffleboard.getTab("PID but better").add("Current CURRENT", currentCURRENT).getEntry();
  GenericEntry stateEntry = Shuffleboard.getTab("PID but better").add("Current stae", "NOTINITIALIZED").getEntry();

  public ArmGrab() {
    WinchMotor.setInverted(true);
    WinchMotor.getPIDController().setFF(kF);
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

  public static States state = States.CLOSING_STARTING_VELOCITY;

  public void setRPM(double rpm){
    WinchMotor.getPIDController().setReference(rpm, ControlType.kVelocity);
  }

  public void setPercentOutput(double percent){
    WinchMotor.getPIDController().setReference(percent, ControlType.kDutyCycle);
  }

  public void stopMotor(){
    targetVelEntry.setDouble(0);
  }

  public double getCurrentOutput(){
    return WinchMotor.getOutputCurrent();
  }

  public void resetPosition(){
    WinchMotor.getEncoder().setPosition(0);
  }

  public double getCurrentPosition(){
    return WinchMotor.getEncoder().getPosition();
  }

  public double getMotorVelocity(){
    return WinchMotor.getEncoder().getVelocity();
  }

  // public void firstVelocityPass(){
  //   if(currentVel >= targetRPM * 0.8) {
  //     state = States.CLOSING_TIME_ELAPSING;
  //   }
  // }

  public void firstCurrentPass(){
    timer.start();
    if(timer.hasElapsed(0.5)){
      timer.stop();
      timer.reset();
      state = States.CLOSING_CURRENT;
    } 
  }

  @Override
  public void periodic(){
    //System.out.println(getCurrentPosition());
    //setCurrentLimit(Constants.TARGET_CURRENT_VALUE);
    //System.out.println("Crashing");
    //System.out.println(state);
    //System.out.println(limitSwitchInput.get());
    currentCURRENT = WinchMotor.getOutputCurrent();
    currentCURRENTEntry.setDouble(currentCURRENT);
    currentVel = WinchMotor.getEncoder().getVelocity();
    currentVelEntry.setDouble(currentVel);



    if(DriverStation.isEnabled()) {
      switch(state) {
        case NOT_INITIALIZED:
          //kF = fEntry.getDouble(kF);
          //targetVelEntry.setDouble(targetRPM);
          stateEntry.setString("Not Initialized");
          state = States.INITIALIZING_OPENING;
          break;

        case INITIALIZING_OPENING:
          targetRPM = -Constants.GRABBER_TARGET_RPM;
          setRPM(targetRPM);
          if(limitSwitchInput.get()){
            state = States.INITIALIZING_STOPPING;
          }

          stateEntry.setString("Initializing-- Opening");
          break;
        case INITIALIZING_STOPPING:
          targetRPM = 0;
          //targetVelEntry.setDouble(targetRPM);
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
          targetRPM = -Constants.GRABBER_TARGET_RPM;
          setRPM(targetRPM);
          if(getCurrentPosition() <= -Constants.GRABBER_MAX_OPEN_POS) {
            targetRPM = 0;
            //targetVelEntry.setDouble(targetRPM);
            setRPM(targetRPM);
            state = States.OPENED;
          }
          break;
        case OPENED:
          stateEntry.setString("Opened");
          break;
        case CLOSING_STARTING_VELOCITY:
          //System.out.println("timer" + timer.get());
          stateEntry.setString("Waiting for Velocity spike");
          // currentCURRENTEntry.setDouble(currentCURRENT);
          // currentVelEntry.setDouble(currentVel);
          //targetRPM = targetVelEntry.getDouble(targetRPM);
          setRPM(Constants.GRABBER_TARGET_RPM);
          if(currentVel >= targetRPM * 0.8) {
            state = States.CLOSING_TIME_ELAPSING;
          }
          break;
        case CLOSING_TIME_ELAPSING:
          stateEntry.setString("Waiting for time lapse");
          firstCurrentPass();
          state = States.CLOSING_CURRENT;
          break;
        case CLOSING_CURRENT:
          stateEntry.setString("Closing current");
          if(RobotContainer.coneMode.getAsBoolean()) {
            if(getCurrentOutput() >= Constants.TARGET_CONE_CURRENT_VALUE) {
              stopMotor();
              setPercentOutput(Constants.GRABBER_PERCENT_OUTPUT);
              ArmGrab.state = States.CLOSED;
            }
          } else {
            if(getCurrentOutput() >= Constants.TARGET_CUBE_CURRENT_VALUE) {
              stopMotor();
              setPercentOutput(Constants.GRABBER_PERCENT_OUTPUT);
              ArmGrab.state = States.CLOSED;
            }
          }
          
          // currentCURRENT = WinchMotor.getOutputCurrent();
          //System.out.println("Current" + currentCURRENT);
          // currentCURRENTEntry.setDouble(currentCURRENT);
          // currentVel = WinchMotor.getEncoder().getVelocity();
          // currentVelEntry.setDouble(currentVel);
          break;
        case CLOSED:
          //targetRPM = 0;
          currentCURRENT = WinchMotor.getOutputCurrent();
          currentCURRENTEntry.setDouble(currentCURRENT);
          targetVelEntry.setDouble(targetRPM);
          stateEntry.setString("Closed");
          break;
        default:
          stateEntry.setString("Why are you in default");
          break;
      }
    //currentCURRENT = 0;
    }
    // System.out.println("timer" + timer.get());
    // targetRPM = targetVelEntry.getDouble(targetRPM);
    // setRPM(targetRPM);
    // currentCURRENT = WinchMotor.getOutputCurrent();
    // System.out.print(currentCURRENT);
    // currentCURRENTEntry.setDouble(currentCURRENT);
    // kP = pEntry.getDouble(kP);
    // kI = iEntry.getDouble(kI);
    // kD = dEntry.getDouble(kD);
    //   kF = fEntry.getDouble(kF);
    //   WinchMotor.getPIDController().setFF(kF);
    //   currentVel = WinchMotor.getEncoder().getVelocity();
    //   currentVelEntry.setDouble(currentVel);
    //   WinchMotor.getPIDController().setP(kP);
  }

}
