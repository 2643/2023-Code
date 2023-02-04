// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.networktables.NetworkTableEntry;

public class BillGates extends SubsystemBase {
  /** Creates a new BillGates. */
  CANSparkMax WinchMotor = new CANSparkMax(Constants.GRABBER_MOTOR_PORT, MotorType.kBrushless);
   
    //NetworkEntry pEntry = Shuffleboard.getTab("PID").add("Proportional", 0).getEntry();
    //GenericEntry iEntry = Shuffleboard.getTab("PID").add("Integral", 0).getEntry();
    //GenericEntry dEntry = Shuffleboard.getTab("PID").add("Derivative", 0).getEntry();
    //GenericEntry targetEntry = Shuffleboard.getTab("PID").add("Velocity", 0).getEntry();
  

    double kF;
    double kP;
    double kI;
    double kD;
    double targetPos;
    double accel;
    double vel;
    double pos;
  public BillGates() {}


  public void setRPM(double rpm){
    WinchMotor.getPIDController().setReference(rpm, ControlType.kVelocity);
  }

  public double getCurrentOutput(){
    return WinchMotor.getAppliedOutput();
  }




  @Override
  public void periodic() {}
  

}
