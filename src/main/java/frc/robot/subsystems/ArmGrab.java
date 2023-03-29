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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class ArmGrab extends SubsystemBase {
  static boolean armGrabInitialized = false;
  Timer timer = new Timer();
  DigitalInput limitSwitchInput = new DigitalInput(Constants.ArmGrab.GRABBER_LIMIT_SWITCH_PORT);
  CANSparkMax WinchMotor = new CANSparkMax(Constants.ArmGrab.GRABBER_MOTOR_PORT, MotorType.kBrushless);
   
  double kP = 0;
  double kF = 0.0000844;
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

  

  static States state = States.CLOSING_STARTING_VELOCITY;

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

  public void setArmGrabState(States state){
    ArmGrab.state = state;
  }

  public States getArmGrabState(){
    return state;
  }

  public boolean getArmGrabInitialized() {
    return armGrabInitialized;
  }

  public void firstCurrentPass(){
    timer.start();
    if(timer.hasElapsed(0.3)){
      timer.stop();
      timer.reset();
      state = States.CLOSING_CURRENT;
    } 
  }

  public boolean isClosed() {
    return state == States.CLOSED || state == States.CLOSING_CURRENT || state == States.CLOSING_STARTING_VELOCITY || state == States.CLOSING_TIME_ELAPSING;
  }

  @Override
  public void periodic(){
    currentCURRENT = WinchMotor.getOutputCurrent();
    currentCURRENTEntry.setDouble(currentCURRENT);
    currentVel = WinchMotor.getEncoder().getVelocity();
    currentVelEntry.setDouble(currentVel);

    if(DriverStation.isEnabled()) {
      switch(state) {
        case NOT_INITIALIZED:
          stateEntry.setString("Not Initialized");
          state = States.INITIALIZING_OPENING;
          break;

        case INITIALIZING_OPENING:
          targetRPM = -Constants.ArmGrab.GRABBER_TARGET_RPM;
          setRPM(targetRPM);
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
          firstCurrentPass();
          //state = States.CLOSING_CURRENT;
          break;
        case CLOSING_CURRENT:
          stateEntry.setString("Closing current");
          if(RobotContainer.coneMode.getAsBoolean()) {
            if(getCurrentOutput() >= Constants.ArmGrab.TARGET_CONE_CURRENT_VALUE) {
              stopMotor();
            if(RobotContainer.coneMode.getAsBoolean()) {
              setPercentOutput(Constants.ArmGrab.GRABBER_PERCENT_OUTPUT);
            } else {
              setPercentOutput(Constants.ArmGrab.GRABBER_CUBE_PERCENT_OUTPUT);
            }
              ArmGrab.state = States.CLOSED;
            }
          } else {
            if(getCurrentOutput() >= Constants.ArmGrab.TARGET_CUBE_CURRENT_VALUE) {
              stopMotor();
              setPercentOutput(Constants.ArmGrab.GRABBER_PERCENT_OUTPUT);
              ArmGrab.state = States.CLOSED;
            }
          }
          break;
        case CLOSED:
          currentCURRENT = WinchMotor.getOutputCurrent();
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
