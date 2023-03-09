// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

  TalonFX leftArmMotor = new TalonFX(Constants.ARM_LIFT_LEFT_MOTOR_PORT);
  TalonFX rightArmMotor = new TalonFX(Constants.ARM_LIFT_RIGHT_MOTOR_PORT);

  DigitalInput limitSwitchInput = new DigitalInput(Constants.LIMIT_SWITCH_PORT_ONE);
  DigitalInput limitSwitchTwo = new DigitalInput(Constants.LIMIT_SWITCH_PORT_TWO);
  
  Timer timer = new Timer();

  GenericEntry pEntry = Shuffleboard.getTab("PID").add("Proportional",0.06).getEntry();
  GenericEntry iEntry = Shuffleboard.getTab("PID").add("Integral",0).getEntry();
  GenericEntry dEntry = Shuffleboard.getTab("PID").add("Derivative",0).getEntry();
  GenericEntry fEntry = Shuffleboard.getTab("PID").add("Feed Forward",0).getEntry();
  GenericEntry targetEntry = Shuffleboard.getTab("PID").add("Target",0).getEntry();
  //GenericEntry currentPosEntry = Shuffleboard.getTab("PID").add("CurrentPosition", 0).getEntry();
  GenericEntry accelEntry = Shuffleboard.getTab("PID").add("Acceleration",1200).getEntry();
  GenericEntry velEntry = Shuffleboard.getTab("PID").add("Velocity",10000).getEntry();
  GenericEntry THL = Shuffleboard.getTab("PID").add("Top hard limit",Constants.TOP_HARD_LIMIT_MOVEPOS).getEntry();
  GenericEntry TSL = Shuffleboard.getTab("PID").add("Top Soft Limit", Constants.TOP_SOFT_LIMIT_MOVEPOS).getEntry();
  GenericEntry BHL = Shuffleboard.getTab("PID").add("Bottom Hard Limit",Constants.BOTTOM_HARD_LIMIT_MOVEPOS).getEntry();
  GenericEntry BSL = Shuffleboard.getTab("PID").add("Bottom Soft Limit",Constants.BOTTOM_SOFT_LIMIT_MOVEPOS).getEntry();
  GenericEntry currentPosEntry = Shuffleboard.getTab("PID").add("Current Pos",0).getEntry();
  GenericEntry PosErrEntry = Shuffleboard.getTab("PID").add("PosErr",0).getEntry();
  GenericEntry targetPosEntry = Shuffleboard.getTab("PID").add("Target Pos",0).getEntry();
  GenericEntry FFPosEntry = Shuffleboard.getTab("PID").add("FF",0).getEntry();
  GenericEntry FFValEntry = Shuffleboard.getTab("PID").add("FFVal",0.04).getEntry();
  GenericEntry deadBandEntry = Shuffleboard.getTab("PID").add("deadBand",0.04).getEntry();
  GenericEntry percentMaxEntry = Shuffleboard.getTab("PID").add("percentMax",0.7).getEntry();
  //GenericEntry coastModeButton = Shuffleboard.getTab("PID").add("coast button",Constants.BOTTOM_SOFT_LIMIT_MOVEPOS).withWidget(BuiltInWidgets.kToggleButton).getEntry();


  double kF;
  double kP;
  double kI;
  double kD;
  double targetPos;
  double accel;
  double vel;
  double pos;
  double softToHardTarget;


  double AuxiliaryFF = 0;

  public ArmLift() {
    leftArmMotor.configFactoryDefault();
    leftArmMotor.selectProfileSlot(0, 0);

    leftArmMotor.config_kF(0, 0, 1);
    leftArmMotor.config_kP(0, 0.06, 1);
    leftArmMotor.config_kI(0, 0, 1);
    leftArmMotor.config_kD(0, 0, 1);
    leftArmMotor.configPeakOutputForward(0.7);
    leftArmMotor.configPeakOutputReverse(-0.7);

    leftArmMotor.configMotionCruiseVelocity(1000, 1);
    leftArmMotor.configMotionAcceleration(1200, 1);
    leftArmMotor.configNeutralDeadband(0.04);
    leftArmMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1);
    leftArmMotor.setInverted(false);
    rightArmMotor.follow(leftArmMotor);
    rightArmMotor.setInverted(InvertType.OpposeMaster);
  }

  //ArmFeedforward test = new ArmFeedforward(0, 0, 0);

  public void movePos(double pos) {
    leftArmMotor.set(TalonFXControlMode.MotionMagic, pos);
  }
  
  public void movePosFF(double pos) {
    leftArmMotor.set(TalonFXControlMode.MotionMagic, pos, DemandType.ArbitraryFeedForward, AuxiliaryFF);
  }

  public void startTimer(){
    timer.reset();
    timer.start();
  }
  public void stopTimer(){
    timer.stop();
  }
  public double getTimer(){
    return timer.get();
  }

  public void destroyMotor() {
    leftArmMotor.DestroyObject();
    leftArmMotor.configFactoryDefault();
  }


  public double getPos() {
    return leftArmMotor.getSelectedSensorPosition();
  }

  public void reset(){
    setPos(0);
    movePos(0);
  }
  public void afterRestMovePos() {
  if(MoveArm.targetPos >= Constants.TOP_HARD_LIMIT_MOVEPOS || MoveArm.targetPos < Constants.BOTTOM_HARD_LIMIT_MOVEPOS) {
    destroyMotor();
  } else if(MoveArm.targetPos > Constants.TOP_SOFT_LIMIT_MOVEPOS) {
    MoveArm.targetPos = Constants.TOP_SOFT_LIMIT_MOVEPOS;
  } else if(MoveArm.targetPos < Constants.BOTTOM_SOFT_LIMIT_MOVEPOS) {
    MoveArm.targetPos = Constants.BOTTOM_SOFT_LIMIT_MOVEPOS;
  }
  movePos(MoveArm.targetPos);
}


  public void setPos(double pos) {
    leftArmMotor.setSelectedSensorPosition(pos);
  }

  public double getVel(){
    return leftArmMotor.getSelectedSensorVelocity();
  }

  public boolean getLimitSwitch() {
    return limitSwitchInput.get();
  }

  public void changeAcceleration(double accel) {
    leftArmMotor.configMotionAcceleration(accel);
  }

  public void changeVelocity(double vel) {
    leftArmMotor.configMotionCruiseVelocity(vel);
  }

  public void speedControl(double percent) {
    leftArmMotor.set(ControlMode.PercentOutput, percent);
  }

  public double getStatorCurrent() {
    return leftArmMotor.getStatorCurrent();
  }

  public void disableMotor() {
    leftArmMotor.set(ControlMode.Disabled, 0);
  }

  public boolean getLimitSwitchTwo(){
    return limitSwitchTwo.get();
  }
  
  @Override
  public void periodic() {
    if(Constants.armInitialized){
      AuxiliaryFF = -FFValEntry.getDouble(0.04) * Math.sin(Math.toRadians((MoveArm.targetPos/Constants.COUNT_PER_DEGREES) + 47));
      movePosFF(MoveArm.targetPos);

      
      currentPosEntry.setDouble(getPos());
      FFPosEntry.setDouble(AuxiliaryFF);
      targetPosEntry.setDouble(MoveArm.targetPos);
      PosErrEntry.setDouble(MoveArm.targetPos-getPos());

      leftArmMotor.config_kP(0, pEntry.getDouble(0.06));
      leftArmMotor.config_kI(0, iEntry.getDouble(0));
      leftArmMotor.config_kD(0, dEntry.getDouble(0));
      leftArmMotor.configNeutralDeadband(deadBandEntry.getDouble(0.04));
      leftArmMotor.configMotionCruiseVelocity(velEntry.getDouble(10000), 1);
      leftArmMotor.configMotionAcceleration(accelEntry.getDouble(1200), 1);
      leftArmMotor.configPeakOutputForward(percentMaxEntry.getDouble(0.7));
      leftArmMotor.configPeakOutputReverse(-percentMaxEntry.getDouble(0.7));



    
      leftArmMotor.setNeutralMode(NeutralMode.Brake);
      rightArmMotor.setNeutralMode(NeutralMode.Brake);
      
    }


    
  }
}