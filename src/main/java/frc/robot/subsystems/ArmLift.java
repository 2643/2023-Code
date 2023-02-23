// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import java.util.TimerTask;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmLift.MoveArm;

public class ArmLift extends SubsystemBase {

  public static enum moveArmJoystick{
    Up,
    Down, 
    Encoder,
    BottomNode,
    MiddleNode,
  }
  /** Creates a new Motor. */
  // Talon motor contoroller
  // TalonFX motor = new TalonFX(3);
  AnalogPotentiometer pot = new AnalogPotentiometer(0, 360, 0);
  TalonFX m_motor = new TalonFX(Constants.ARM_LIFT_MOTOR_PORT);
  DigitalInput limitSwitchInput = new DigitalInput(Constants.LIMIT_SWITCH_PORT_ONE);
  Timer timer = new Timer();


  DigitalInput limitSwitchTwo = new DigitalInput(Constants.LIMIT_SWITCH_PORT_TWO); 
  GenericEntry pEntry = Shuffleboard.getTab("PID").add("Proportional",0).getEntry();
  GenericEntry iEntry = Shuffleboard.getTab("PID").add("Integral",0).getEntry();
  GenericEntry dEntry = Shuffleboard.getTab("PID").add("Derivative",0).getEntry();
  GenericEntry fEntry = Shuffleboard.getTab("PID").add("Feed Forward",0).getEntry();
  GenericEntry targetEntry = Shuffleboard.getTab("PID").add("Target",0).getEntry();
  //GenericEntry currentPosEntry = Shuffleboard.getTab("PID").add("CurrentPosition", 0).getEntry();
  GenericEntry accelEntry = Shuffleboard.getTab("PID").add("Acceleration",0).getEntry();
  GenericEntry velEntry = Shuffleboard.getTab("PID").add("Velocity",0).getEntry();
  GenericEntry THL = Shuffleboard.getTab("PID").add("Top hard limit",Constants.TOP_HARD_LIMIT_MOVEPOS).getEntry();
  GenericEntry TSL = Shuffleboard.getTab("PID").add("Top Soft Limit", Constants.TOP_SOFT_LIMIT_MOVEPOS).getEntry();
  GenericEntry BHL = Shuffleboard.getTab("PID").add("Bottom Hard Limit",Constants.BOTTOM_HARD_LIMIT_MOVEPOS).getEntry();
  GenericEntry BSL = Shuffleboard.getTab("PID").add("Bottom Soft Limit",Constants.BOTTOM_SOFT_LIMIT_MOVEPOS).getEntry();
  GenericEntry currentPosEntry = Shuffleboard.getTab("PID").add("Current Pos",Constants.BOTTOM_SOFT_LIMIT_MOVEPOS).getEntry();
  GenericEntry targetPosEntry = Shuffleboard.getTab("PID").add("Target Pos",Constants.BOTTOM_SOFT_LIMIT_MOVEPOS).getEntry();


  double kF;
  double kP;
  double kI;
  double kD;
  double targetPos;
  double accel;
  double vel;
  double pos;
  double softToHardTarget;

  // CANSparkMax
  // CANSparkMax rightFrontmotor = new CANSparkMax(1, MotorType.kBrushless);
  public ArmLift() {
    m_motor.configFactoryDefault();
    m_motor.selectProfileSlot(0, 0);
    // m_motor.setSelectedSensorPosition(pos, 0, 0)
    m_motor.config_kF(0, 0, 1);
    m_motor.config_kP(0, 0.05, 1);
    m_motor.config_kI(0, 0, 1);
    m_motor.config_kD(0, 0, 1);
    m_motor.configMotionCruiseVelocity(100000, 1);
    m_motor.configMotionAcceleration(10000, 1);
    m_motor.configNeutralDeadband(0.04);
    m_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1);
    m_motor.setInverted(true);

    // makes the wheels go the other direction
    // rightFrontmotor.setInverted(true);
  }
  // public void setSpeed(double speed) {
  // rightFrontmotor.getPIDController().setReference(speed,
  // ControlType.kDutyCycle);
  // }

  // CANSparkMax leftFrontmotor = new CANSparkMax(3, MotorType.kBrushless);
  // public void setSpeed2(double speed) {
  // leftFrontmotor.getPIDController().setReference(speed,
  // ControlType.kDutyCycle);

  // }
  // Talon
  // public static final TalonFXControlMode MotionProfile =
  // TalonFXControlMode.MotionMagic;
  public void movePos(double pos) {
    m_motor.set(TalonFXControlMode.MotionMagic, pos);
  }
  
  public void starttimer(){
    timer.reset();
    timer.start();
  }
  public void stoptimer(){
    timer.stop();
  }
  public double gettimer(){
    return timer.get();
  }

  public void destroyObject() {
    m_motor.DestroyObject();
    m_motor.configFactoryDefault();
   
  }


  public double getPos() {
    return m_motor.getSelectedSensorPosition();
  }

  public void reset(){
    setPos(0);
    movePos(0);
  }
  public void afterRestMovePos() {
  //   if (getPos() < Constants.TOP_SOFT_LIMIT_MOVEPOS && getPos() > Constants.BOTTOM_SOFT_LIMIT_MOVEPOS) {
  //     movePos(movePos);
  //     System.out.println(getPos());
  //   } else {
  //     System.out.println(getPos());
  //     if(getPos() > Constants.TOP_HARD_LIMIT_MOVEPOS || getPos() < Constants.BOTTOM_HARD_LIMIT_MOVEPOS) {
  //       destroyObject();
  //     } else if(getPos() > Constants.TOP_SOFT_LIMIT_MOVEPOS && getPos() > MoveArm.targetPos) {
  //       movePos(movePos);
  //     } else if(getPos() < Constants.BOTTOM_SOFT_LIMIT_MOVEPOS && getPos() < MoveArm.targetPos){
  //       movePos(movePos);
  //     }
  // }

  // if (getPos() < Constants.TOP_SOFT_LIMIT_MOVEPOS && getPos() > Constants.BOTTOM_SOFT_LIMIT_MOVEPOS) {
  //   movePos(movePos);
  //   System.out.println(getPos());
  // } else {
  //   System.out.println(getPos());
  //   if(getPos() >= Constants.TOP_HARD_LIMIT_MOVEPOS || getPos() < Constants.BOTTOM_HARD_LIMIT_MOVEPOS) {
  //     destroyObject();
  //   } else if(MoveArm.targetPos > Constants.TOP_SOFT_LIMIT_MOVEPOS) {
  //     MoveArm.targetPos = Constants.TOP_SOFT_LIMIT_MOVEPOS;
  //     movePos(MoveArm.targetPos);
  //   } else if(MoveArm.targetPos < Constants.BOTTOM_SOFT_LIMIT_MOVEPOS){
  //     MoveArm.targetPos = Constants.BOTTOM_SOFT_LIMIT_MOVEPOS;
  //     movePos(MoveArm.targetPos);
  //   }
  if(MoveArm.targetPos >= Constants.TOP_HARD_LIMIT_MOVEPOS || MoveArm.targetPos < Constants.BOTTOM_HARD_LIMIT_MOVEPOS) {
    destroyObject();
  } else if(MoveArm.targetPos > Constants.TOP_SOFT_LIMIT_MOVEPOS) {
    MoveArm.targetPos = Constants.TOP_SOFT_LIMIT_MOVEPOS;
  } else if(MoveArm.targetPos < Constants.BOTTOM_SOFT_LIMIT_MOVEPOS) {
    MoveArm.targetPos = Constants.BOTTOM_SOFT_LIMIT_MOVEPOS;
  }
  movePos(MoveArm.targetPos);
}


  public void setPos(double pos) {
    m_motor.setSelectedSensorPosition(pos, 0, 1);
  }

  public double getvel(){
    return m_motor.getSelectedSensorVelocity();
  }

  public boolean getLimitSwitch() {
    return limitSwitchInput.get();
  }

  public void changeAcceleration(double accel) {
    m_motor.configMotionAcceleration(accel);
  }

  public void changeVelocity(double vel) {
    m_motor.configMotionCruiseVelocity(vel);
  }

  public void speedControl(double percent) {
    m_motor.set(ControlMode.PercentOutput, percent);
  }

  public double getStatorCurrent() {
    return m_motor.getStatorCurrent();
  }

  public void disablemotor() {
    m_motor.set(ControlMode.Disabled, 0);
  }

  public boolean getLimitSwitchTwo(){
    return limitSwitchTwo.get();
  }
  
  public double stringPotget(){
    return pot.get();
  }

  @Override
  public void periodic() {
    currentPosEntry.setDouble(getPos());
    targetPosEntry.setDouble(MoveArm.targetPos);
    System.out.println(limitSwitchTwo.get());
  }
}