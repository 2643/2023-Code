// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import java.util.TimerTask;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
//import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
  DigitalInput limitSwitchInput = new DigitalInput(Constants.LIMIT_SWITCH_PORT);
  Timer timer = new Timer();
  DigitalInput limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_PORT_TWO); 
  // GenericEntry pEntry = Shuffleboard.getTab("PID").add("Proportional",
  // 0).getEntry();
  // GenericEntry iEntry = Shuffleboard.getTab("PID").add("Integral",
  // 0).getEntry();
  // GenericEntry dEntry = Shuffleboard.getTab("PID").add("Derivative",
  // 0).getEntry();
  // GenericEntry fEntry = Shuffleboard.getTab("PID").add("Feed Forward",
  // 0).getEntry();
  // GenericEntry targetEntry = Shuffleboard.getTab("PID").add("Target",
  // 0).getEntry();
  // GenericEntry currentPosEntry = Shuffleboard.getTab("PID").add("Current
  // Position", 0).getEntry();
  // GenericEntry accelEntry = Shuffleboard.getTab("PID").add("Acceleration",
  // 0).getEntry();
  // GenericEntry velEntry = Shuffleboard.getTab("PID").add("Velocity",
  // 0).getEntry();
  // SimpleWidget buttontarget = Shuffleboard.getTab("PID").add("Button",
  // false).withWidget(BuiltInWidgets.kToggleButton);

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
    m_motor.config_kI(0, 0.0000001, 1);
    m_motor.config_kD(0, 0, 1);
    m_motor.configMotionCruiseVelocity(100000, 1);
    m_motor.configMotionAcceleration(10000, 1);
    m_motor.configNeutralDeadband(0.04);
    m_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1);

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
  public void afterRestMovePos(double movePos) {
    if (getPos() < Constants.TOP_SOFT_LIMIT_MOVEPOS && getPos() > Constants.BOTTOM_SOFT_LIMIT_MOVEPOS) {
      movePos(movePos);
    } else {
      if(getPos() >= Constants.TOP_HARD_LIMIT_MOVEPOS && getPos() > Constants.BOTTOM_HARD_LIMIT_MOVEPOS) {
        destroyObject();
      } else if(getPos() > Constants.TOP_SOFT_LIMIT_MOVEPOS && getPos() > MoveArm.targetPos) {
        movePos(movePos);
      } else if(getPos() < Constants.BOTTOM_SOFT_LIMIT_MOVEPOS && getPos() < MoveArm.targetPos){
        movePos(movePos);
      }
    }

    // } else if (getPos() >= Constants.TOP_HARD_LIMIT_MOVEPOS && getPos() > Constants.BOTTOM_HARD_LIMIT_MOVEPOS) {
    //   destroyObject();
    // } else if()
  
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
    return limitSwitch.get();
    
  }
  
  public double stringPotget(){
    return pot.get();
  }
  // int count = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Talon
    // motion magic position control
    // if(!buttontarget.getEntry().getBoolean(false)) {
    // targetPos = targetEntry.getDouble(0);
    // movePos(targetPos);
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

    // }
    // else{
    // setPos(0);
    // movePos(0);
    // //targetPos = targetEntry.getDouble(0);
    // }
    // CANSparkMax
    // setSpeed(0.1);
    // setSpeed2(0.1);
    // System.out.println(limitSwitchInput.get());
  }
}