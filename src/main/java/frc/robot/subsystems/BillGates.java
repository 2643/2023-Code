// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class BillGates extends SubsystemBase {
  /** Creates a new BillGates. */
  Timer timer = new Timer();
  DigitalInput GlimitSwitchInput = new DigitalInput(Constants.GRABBER_LIMIT_SWITCH_PORT);
  CANSparkMax WinchMotor = new CANSparkMax(Constants.GRABBER_MOTOR_PORT, MotorType.kBrushless);
   
    double kP = 0.0000;
    double kF = 0.0000844;
    double kI;
    double kD;
    double rpm;
    double currentVel;
    double currentPos;
    double currentCURRENT = 0.0;
    double targetRPM = 1000;
    boolean FirstCurrent = true;

    //GenericEntry pEntry = Shuffleboard.getTab("PID but better").add("Proportional", kP).getEntry();
    //GenericEntry iEntry = Shuffleboard.getTab("PID but better").add("Integral", kI).getEntry();
    //GenericEntry dEntry = Shuffleboard.getTab("PID but better").add("Derivative", kD).getEntry();
    GenericEntry fEntry = Shuffleboard.getTab("PID but better").add("Feed Forward", kF).getEntry();
    GenericEntry targetVelEntry = Shuffleboard.getTab("PID but better").add("Target", targetRPM).getEntry();
    GenericEntry currentVelEntry = Shuffleboard.getTab("PID but better").add("Current velocity", currentVel).getEntry();
    GenericEntry currentPosEntry = Shuffleboard.getTab("PID but better").add("Current Position", currentPos).getEntry();
    GenericEntry limitSwitchDetectEntry = Shuffleboard.getTab("PID but better").add("Limit Switch Detect", GlimitSwitchInput.get()).getEntry();
    GenericEntry currentCURRENTEntry = Shuffleboard.getTab("PID but better").add("Current CURRENT", currentCURRENT).getEntry();

  public BillGates() {
  }
  public enum States{
    NOTINITIALIZED,
    INITIALIZINGOPENING,
    INITIALIZINGSTOPPING,
    INITIALIZED,
    OPENING,
    OPENED,
    CLOSINGSTART,
    CLOSINGCURRENT,
    CLOSED;

  }
public static States state = States.CLOSINGSTART;

  public void setRPM(double rpm){
    WinchMotor.getPIDController().setReference(rpm, ControlType.kVelocity);
  }

  public void setCurrentLimit(double amps){
    WinchMotor.getPIDController().setReference(amps, ControlType.kCurrent);
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

  public void firstCurrentPass(){
    //timer.start();
    //System.out.println();
    if(currentVel >= targetRPM*0.8){
      FirstCurrent = false;
      timer.stop();
      timer.reset();
      state = States.CLOSINGCURRENT;
      System.out.println(state);
    } 
  }

  @Override
  public void periodic(){
    getCurrentPosition();
    currentPosEntry.setDouble(getCurrentPosition());
    //setCurrentLimit(Constants.TARGET_CURRENT_VALUE);
    //System.out.println("Crashing");
    //System.out.println(state);
    limitSwitchDetectEntry.setBoolean((GlimitSwitchInput.get()));
    currentCURRENT = WinchMotor.getOutputCurrent();
    currentCURRENTEntry.setDouble(currentCURRENT);
    currentVel = WinchMotor.getEncoder().getVelocity();
    currentVelEntry.setDouble(currentVel);

    if(DriverStation.isEnabled()){
      switch(state){
        case NOTINITIALIZED:
          //kF = fEntry.getDouble(kF);
          WinchMotor.setInverted(true);
          WinchMotor.getPIDController().setFF(kF);
          resetPosition();
          targetRPM = -1000;
          //targetVelEntry.setDouble(targetRPM);
          //System.out.println("Not Initialized");
          state = States.INITIALIZINGOPENING;
          System.out.print(state);
          break;
        case INITIALIZINGOPENING:
          setRPM(targetRPM);
          if(GlimitSwitchInput.get() == true){
            state = States.INITIALIZINGSTOPPING;
            System.out.println(state);
          }
          //System.out.println("Initializing-- Opening");
          break;
        case INITIALIZINGSTOPPING:
          targetRPM = 0;
          //targetVelEntry.setDouble(targetRPM);
          setRPM(targetRPM);
          resetPosition();
          state = States.INITIALIZED;
          System.out.println(state);
          //System.out.println("Initializing-- Stopping");
          break;
        case INITIALIZED:
          //System.out.println("Initialized");
          state = States.OPENING;
          System.out.println(state);
          break;
        case OPENING:
          targetRPM = -1000;
          setRPM(targetRPM);
          if(getCurrentPosition() >= Constants.GRABBER_MAX_OPEN_POS || getCurrentPosition() <= Constants.GRABBER_MAX_OPEN_POS*-1){
            targetRPM = 0;
            //targetVelEntry.setDouble(targetRPM);
            setRPM(targetRPM);
            state = state.OPENED;
            System.out.println(state);
          }
         //System.out.println("Opening");
          break;
        case OPENED:
          //System.out.println("Opened");
          break;
        case CLOSINGSTART:
          WinchMotor.setInverted(true);
          System.out.println(FirstCurrent);
          //System.out.println("timer" + timer.get());
          // currentCURRENTEntry.setDouble(currentCURRENT);
          // currentVelEntry.setDouble(currentVel);
          //targetRPM = targetVelEntry.getDouble(targetRPM);
          setRPM(2000);
          break;
        case CLOSINGCURRENT:
          // currentCURRENT = WinchMotor.getOutputCurrent();
          // //System.out.println("Current" + currentCURRENT);
          // currentCURRENTEntry.setDouble(currentCURRENT);
          // currentVel = WinchMotor.getEncoder().getVelocity();
          // currentVelEntry.setDouble(currentVel);
          break;
        case CLOSED:
          targetRPM = 0;
          currentCURRENT = WinchMotor.getOutputCurrent();
          currentCURRENTEntry.setDouble(currentCURRENT);
          targetVelEntry.setDouble(targetRPM);
          setRPM(0);
          setCurrentLimit(Constants.TARGET_CURRENT_VALUE);
          //System.out.println("Closed");
          break;
        default:
          System.out.println("Why are you in default");
          break;
    }
    currentCURRENT = 0;
    }
    //System.out.println(state);
    // System.out.println(FirstCurrent);
    // System.out.println("timer" + timer.get());
    // targetRPM = targetVelEntry.getDouble(targetRPM);
    // setRPM(targetRPM);
    // currentCURRENT = WinchMotor.getOutputCurrent();
    // System.out.print(currentCURRENT);
    // currentCURRENTEntry.setDouble(currentCURRENT);
    //   //kP = pEntry.getDouble(kP);
    //   //kI = iEntry.getDouble(kI);
    //   //kD = dEntry.getDouble(kD);
    //   kF = fEntry.getDouble(kF);
    //   WinchMotor.getPIDController().setFF(kF);
    //   currentVel = WinchMotor.getEncoder().getVelocity();
    //   currentVelEntry.setDouble(currentVel);
    //   WinchMotor.getPIDController().setP(kP);
  }

}
