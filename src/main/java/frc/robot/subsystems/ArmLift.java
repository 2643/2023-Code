// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

//import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmLift extends SubsystemBase {
  /** Creates a new Motor. */
   // Talon motor contoroller
   //TalonFX motor = new TalonFX(3);
   TalonFX m_motor = new TalonFX(Constants.ARM_LIFT_MOTOR_PORT);
   DigitalInput limitSwitchInput = new DigitalInput(Constants.LIMIT_SWITCH_PORT);
  
   
  //  GenericEntry pEntry = Shuffleboard.getTab("PID").add("Proportional", 0).getEntry();
  //  GenericEntry iEntry = Shuffleboard.getTab("PID").add("Integral", 0).getEntry();
  //  GenericEntry dEntry = Shuffleboard.getTab("PID").add("Derivative", 0).getEntry();
  //  GenericEntry fEntry = Shuffleboard.getTab("PID").add("Feed Forward", 0).getEntry();
  //  GenericEntry targetEntry = Shuffleboard.getTab("PID").add("Target", 0).getEntry();
  //  GenericEntry currentPosEntry = Shuffleboard.getTab("PID").add("Current Position", 0).getEntry();
  // GenericEntry accelEntry = Shuffleboard.getTab("PID").add("Acceleration", 0).getEntry();
  // GenericEntry velEntry = Shuffleboard.getTab("PID").add("Velocity", 0).getEntry();
 // SimpleWidget buttontarget = Shuffleboard.getTab("PID").add("Button", false).withWidget(BuiltInWidgets.kToggleButton);
  

double kF;
double kP;
double kI;
double kD;
double targetPos;
double accel;
double vel;
double pos;

   


   //CANSparkMax

  //CANSparkMax rightFrontmotor = new CANSparkMax(1, MotorType.kBrushless);

  public ArmLift() {
    
    m_motor.config_kF(0, 0, 1);
    m_motor.config_kP(0, 0.054, 1);
    m_motor.config_kI(0, 0, 1);
    m_motor.config_kD(0 , 0, 1);
    m_motor.configMotionCruiseVelocity(100000, 1);
    m_motor.configMotionAcceleration(10000, 1);
    m_motor.configNeutralDeadband(0.04);
    
    //makes the wheels go the other direction 
    // rightFrontmotor.setInverted(true);
  }

  // public void setSpeed(double speed) {
  //   rightFrontmotor.getPIDController().setReference(speed, ControlType.kDutyCycle);
  // }

  
  //   CANSparkMax leftFrontmotor = new CANSparkMax(3, MotorType.kBrushless);
  // public void setSpeed2(double speed) {
  //   leftFrontmotor.getPIDController().setReference(speed, ControlType.kDutyCycle);
 
  // }

//Talon
//public static final TalonFXControlMode MotionProfile = TalonFXControlMode.MotionMagic;

   public void movePos(double pos) {
    m_motor.set(TalonFXControlMode.MotionMagic, pos);
  } 
  public void setPos(double pos){
    m_motor.setSelectedSensorPosition(pos);
  }
  public boolean getLimitSwitch(){
    return limitSwitchInput.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
//Talon

    //motion magic position control
   
    // targetPos = targetEntry.getDouble(0);
    // setPos(targetPos);

    // currentPosEntry.setDouble(m_motor.getSelectedSensorPosition());
    

    // //m_motor.getPIDController().setReference(speed);
    // kP = pEntry.getDouble(0);
    // kI = iEntry.getDouble(0);
    // kD = dEntry.getDouble(0);
    // kF = fEntry.getDouble(0);
    
    // accel = accelEntry.getDouble(0);
    // vel = velEntry.getDouble(0);

    // m_motor.configMotionAcceleration(accel, 1);
    // m_motor.configMotionCruiseVelocity(vel, 1);

    // m_motor.config_kP(0, kP, 1);
    // m_motor.config_kI(0, kI, 1);
    // m_motor.config_kD(0, kD, 1);
    // m_motor.config_kF(0, kF, 1);

  
    // System.out.println(buttontarget.getEntry().getBoolean(false));
    
    // if(buttontarget.getEntry().getBoolean(false)){
    //   targetPos = targetEntry.getDouble(0);
    }
    // CANSparkMax
    // setSpeed(0.1);
    // setSpeed2(0.1);
 }

