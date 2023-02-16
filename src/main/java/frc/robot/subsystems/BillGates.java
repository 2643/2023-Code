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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class BillGates extends SubsystemBase {
  /** Creates a new BillGates. */
  Timer timer = new Timer();
  CANSparkMax WinchMotor = new CANSparkMax(Constants.GRABBER_MOTOR_PORT, MotorType.kBrushless);
   
    double kP = 0.0000;
    double kF = 0.00049;
    double kI;
    double kD;
    double rpm;
    double currentVel;
    double currentCURRENT = 0.0;
    double targetRPM = 1000;
    boolean FirstCurrent = true;

    GenericEntry pEntry = Shuffleboard.getTab("PID but better").add("Proportional", kP).getEntry();
    GenericEntry iEntry = Shuffleboard.getTab("PID but better").add("Integral", kI).getEntry();
    GenericEntry dEntry = Shuffleboard.getTab("PID but better").add("Derivative", kD).getEntry();
    GenericEntry fEntry = Shuffleboard.getTab("PID but better").add("Feed Forward", kF).getEntry();
    GenericEntry targetVelEntry = Shuffleboard.getTab("PID but better").add("Target", targetRPM).getEntry();
    GenericEntry currentVelEntry = Shuffleboard.getTab("PID but better").add("Current velocity", currentVel).getEntry();
    GenericEntry currentCURRENTEntry = Shuffleboard.getTab("PID but better").add("Current CURRENT", currentCURRENT).getEntry();
  public BillGates() {}


  public void setRPM(double rpm){
    WinchMotor.getPIDController().setReference(rpm, ControlType.kVelocity);
  }

  public void stopMotor(){
    targetVelEntry.setDouble(0);
  }

  public double getCurrentOutput(){
    return WinchMotor.getAppliedOutput();
  }

  public double getMotorVelocity(){
    return WinchMotor.getEncoder().getVelocity();
  }

  public void firstCurrentPass(){
    timer.start();
    if(timer.hasElapsed(3)){
      FirstCurrent = false;
      timer.stop();
      timer.reset();
    }
  }

  @Override
  public void periodic(){
    System.out.println(FirstCurrent);
    System.out.println("timer" + timer.get());
    targetRPM = targetVelEntry.getDouble(targetRPM);
    setRPM(targetRPM);
    currentCURRENT = WinchMotor.getOutputCurrent();
    System.out.print(currentCURRENT);
    currentCURRENTEntry.setDouble(currentCURRENT);
      //kP = pEntry.getDouble(kP);
      //kI = iEntry.getDouble(kI);
      //kD = dEntry.getDouble(kD);
      kF = fEntry.getDouble(kF);
      WinchMotor.getPIDController().setFF(kF);
      currentVel = WinchMotor.getEncoder().getVelocity();
      currentVelEntry.setDouble(currentVel);
      WinchMotor.getPIDController().setP(kP);
  }

}
