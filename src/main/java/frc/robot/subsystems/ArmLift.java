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
//import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
//import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmLift.*;
public class ArmLift extends SubsystemBase {

  public static enum moveArmJoystick{
    Up,
    Down, 
    Encoder
  }

  public static enum ArmLiftStates {
    NOT_INITIALIZED,
    INITIALIZING_CALLED,
    INITIALIZING,
    INITIALIZED
  }

  public static enum positionStates{
    REST,
    PICKUP,
    CONE,
    CUBE,
    CHARGING_STATION,
    FLOOR
  }

  //public static EncoderState encoderState = EncoderState.REST;
  static ArmLiftStates ArmLiftState = ArmLiftStates.NOT_INITIALIZED;
  double AuxiliaryFF = 0;
  static double currentArmEncoderPos = 0;//armPlacement(RobotContainer.operatorBoard.getRawAxis(Constants.ArmLift.ENCODER_PORT));
  static boolean changedEncoderPlacement = false;
  //motors
  TalonFX leftArmMotor = new TalonFX(Constants.ArmLift.ARM_LIFT_LEFT_MOTOR_PORT);
  TalonFX rightArmMotor = new TalonFX(Constants.ArmLift.ARM_LIFT_RIGHT_MOTOR_PORT);

  //limit switches
  DigitalInput limitSwitchInput = new DigitalInput(Constants.ArmLift.LIMIT_SWITCH_PORT_ONE);
  DigitalInput limitSwitchTwo = new DigitalInput(Constants.ArmLift.LIMIT_SWITCH_PORT_TWO);
  
  Timer timer = new Timer();

  //Shuffleboard

  //PID
  // GenericEntry pEntry = Shuffleboard.getTab("PID").add("Proportional",0.12).getEntry();
  // GenericEntry iEntry = Shuffleboard.getTab("PID").add("Integral",0).getEntry();
  // GenericEntry dEntry = Shuffleboard.getTab("PID").add("Derivative",0).getEntry();

  //motion magic velocity and acceleration
  // GenericEntry accelEntry = Shuffleboard.getTab("PID").add("Acceleration",17000).getEntry();
  // GenericEntry velEntry = Shuffleboard.getTab("PID").add("Velocity", 8533).getEntry();

  //limits
  // GenericEntry THL = Shuffleboard.getTab("PID").add("Top hard limit",Constants.TOP_HARD_LIMIT_MOVEPOS).getEntry();
  // GenericEntry TSL = Shuffleboard.getTab("PID").add("Top Soft Limit", Constants.TOP_SOFT_LIMIT_MOVEPOS).getEntry();
  // GenericEntry BHL = Shuffleboard.getTab("PID").add("Bottom Hard Limit",Constants.BOTTOM_HARD_LIMIT_MOVEPOS).getEntry();
  // GenericEntry BSL = Shuffleboard.getTab("PID").add("Bottom Soft Limit",Constants.BOTTOM_SOFT_LIMIT_MOVEPOS).getEntry();

  //position values
  GenericEntry currentPosEntry = Shuffleboard.getTab("Debug").add("ArmLift-Current Pos",0).getEntry();
  // GenericEntry PosErrEntry = Shuffleboard.getTab("PID").add("PosErr",0).getEntry();
  GenericEntry targetPosEntry = Shuffleboard.getTab("Debug").add("ArmLift-Target Pos",0).getEntry();

  //auxiliary feed forward
  // GenericEntry FFPosEntry = Shuffleboard.getTab("PID").add("FF",0).getEntry();
  // GenericEntry FFValEntry = Shuffleboard.getTab("PID").add("FFVal",0.019).getEntry();

  //deadband and percent max and min
  // GenericEntry deadBandEntry = Shuffleboard.getTab("PID").add("deadBand",0.005).getEntry();
  // GenericEntry percentMaxEntry = Shuffleboard.getTab("PID").add("percentMax",0.7).getEntry();
  // GenericEntry percentMinEntry = Shuffleboard.getTab("PID").add("percentMin",0.7).getEntry();

  //buttons positional movements
  // SimpleWidget secondTargetButtonWidget = Shuffleboard.getTab("PID").add("Second Button",false).withWidget(BuiltInWidgets.kToggleButton);
  // SimpleWidget firstTargetButtonWidget = Shuffleboard.getTab("PID").add("First Button",false).withWidget(BuiltInWidgets.kToggleButton);
  SimpleWidget coastMode = Shuffleboard.getTab("Debug").add("Remove Brake Mode",false).withWidget(BuiltInWidgets.kToggleButton);

  // GenericEntry firstPosEntry = Shuffleboard.getTab("PID").add("first position", 0).getEntry();
  // GenericEntry secondPosEntry = Shuffleboard.getTab("PID").add("second position", 0).getEntry();
  GenericEntry percentOutputEntry = Shuffleboard.getTab("Debug").add("ArmLift-Percent Output current", 0).getEntry();

  double kP = 0.12;
  double kI = 0;
  double kD = 0;
  double maxPercentOutput = 0.7;
  int mainSlotID = 0;
  double maxVel = 8533;
  double maxAccel = 17000;
  double deadBand = 0.005;

  // double targetPos;
  // double accel;
  // double pos;
  // double softToHardTarget;


  public ArmLift() {
    leftArmMotor.configFactoryDefault();
    rightArmMotor.configFactoryDefault();

    leftArmMotor.setNeutralMode(NeutralMode.Brake);
    rightArmMotor.setNeutralMode(NeutralMode.Brake);
    leftArmMotor.selectProfileSlot(0, 0);

    leftArmMotor.config_kP(mainSlotID, kP, 1);
    leftArmMotor.config_kI(mainSlotID, kI, 1);
    leftArmMotor.config_kD(mainSlotID, kD, 1);
    leftArmMotor.configPeakOutputForward(maxPercentOutput);
    leftArmMotor.configPeakOutputReverse(-maxPercentOutput);

    leftArmMotor.configMotionCruiseVelocity(maxVel, 1);
    leftArmMotor.configMotionAcceleration(maxAccel, 1);
    leftArmMotor.configNeutralDeadband(deadBand);
    leftArmMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1);
    leftArmMotor.setInverted(false);
    rightArmMotor.follow(leftArmMotor);
    rightArmMotor.setInverted(InvertType.OpposeMaster);
  }

  public void movePos(double pos) {
    leftArmMotor.set(TalonFXControlMode.MotionMagic, pos);
  }
  
  public void movePosFF(double pos) {
    if(MoveArm.targetPos < Constants.ArmLift.TOP_HARD_LIMIT_MOVEPOS || MoveArm.targetPos > Constants.ArmLift.BOTTOM_HARD_LIMIT_MOVEPOS) {
      destroyMotor();
    } else if(MoveArm.targetPos < Constants.ArmLift.TOP_SOFT_LIMIT_MOVEPOS) {
      MoveArm.targetPos = Constants.ArmLift.TOP_SOFT_LIMIT_MOVEPOS + 1000;
    } else if(MoveArm.targetPos > Constants.ArmLift.BOTTOM_SOFT_LIMIT_MOVEPOS) {
      MoveArm.targetPos = Constants.ArmLift.BOTTOM_SOFT_LIMIT_MOVEPOS - 1000;
    }
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

  public ArmLiftStates getArmLiftState() {
    return ArmLiftState;
  }

  public boolean changedEncoderPlacement() {
    return changedEncoderPlacement;
  }

  public void setChangedEncoderPlacement(boolean changed) {
    changedEncoderPlacement = changed;
  }

  public void setArmLiftState(ArmLiftStates state) {
    ArmLiftState = state;
  }

  public double getCurrentEncoderPos() {
    return currentArmEncoderPos;
  }

  public double getPos() {
    return leftArmMotor.getSelectedSensorPosition();
  }

  public void reset(){
    setPos(0);
    movePos(0);
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

  public static double armPlacement(double ctrlValue) {
    if (ctrlValue >= 0.8) {
      return Constants.ArmLift.REST;
    } else if (ctrlValue >= 0.6) {
      return Constants.ArmLift.FLOOR;
    } else if (ctrlValue >= 0.35) {
      return Constants.ArmLift.CHARGING_STATION;
    } else if (ctrlValue >= 0.1){
      return Constants.ArmLift.CUBE;
    } else if (ctrlValue >= 0){
      return Constants.ArmLift.CONE;
    } else if(ctrlValue > -0.5){
      return Constants.ArmLift.PICKUP;
    } else{
      return Constants.ArmLift.REST;
    }
  }
  
  @Override
  public void periodic() {
    if(coastMode.getEntry().getBoolean(false)) {
      leftArmMotor.setNeutralMode(NeutralMode.Coast);
      rightArmMotor.setNeutralMode(NeutralMode.Coast);
    } else {
      leftArmMotor.setNeutralMode(NeutralMode.Brake);
      rightArmMotor.setNeutralMode(NeutralMode.Brake);
    }
    currentPosEntry.setDouble(getPos());
    targetPosEntry.setDouble(MoveArm.targetPos);
    percentOutputEntry.setDouble(leftArmMotor.getMotorOutputPercent());


    AuxiliaryFF = -0.019 * Math.sin(Math.toRadians((getPos()/Constants.ArmLift.COUNT_PER_DEGREES) + 47));

    if(DriverStation.isEnabled()) {
      switch(ArmLiftState) {
        case NOT_INITIALIZED:
          //movePosFF(MoveArm.targetPos);
          if(getLimitSwitchTwo()) {
            ArmLiftState = ArmLiftStates.INITIALIZING_CALLED;
          }
          break;
        case INITIALIZING_CALLED:
          break;
        case INITIALIZING:
          currentArmEncoderPos = armPlacement(RobotContainer.operatorBoard.getRawAxis(Constants.ArmLift.ENCODER_PORT));
          break;
        case INITIALIZED:
          movePosFF(MoveArm.targetPos);
          if(armPlacement(RobotContainer.operatorBoard.getRawAxis(Constants.ArmLift.ENCODER_PORT)) != currentArmEncoderPos) {
            currentArmEncoderPos = armPlacement(RobotContainer.operatorBoard.getRawAxis(Constants.ArmLift.ENCODER_PORT));
            changedEncoderPlacement = true;
          }
          // currentPosEntry.setDouble(getPos());
          // FFPosEntry.setDouble(AuxiliaryFF);
          // targetPosEntry.setDouble(MoveArm.targetPos);
          // PosErrEntry.setDouble(MoveArm.targetPos-getPos());
          // percentOutputEntry.setDouble(leftArmMotor.getMotorOutputPercent());
          leftArmMotor.setNeutralMode(NeutralMode.Brake);
          rightArmMotor.setNeutralMode(NeutralMode.Brake);
          break;

          // leftArmMotor.config_kP(0, pEntry.getDouble(0.12));
          // leftArmMotor.config_kI(0, iEntry.getDouble(0));
          // leftArmMotor.config_kD(0, dEntry.getDouble(0));
          // leftArmMotor.configNeutralDeadband(deadBandEntry.getDouble(0.005));
          // leftArmMotor.configMotionCruiseVelocity(velEntry.getDouble(8533.333333), 1);
          // leftArmMotor.configMotionAcceleration(accelEntry.getDouble(17793.57816), 1);
          // leftArmMotor.configPeakOutputForward(percentMaxEntry.getDouble(0.7));
          // leftArmMotor.configPeakOutputReverse(-percentMaxEntry.getDouble(0.7));
      
          // if(firstTargetButtonWidget.getEntry().getBoolean(false)) {
          //   MoveArm.targetPos = firstPosEntry.getDouble(0);
          // } else if(secondTargetButtonWidget.getEntry().getBoolean(false)) {
          //   MoveArm.targetPos = secondPosEntry.getDouble(0);
          // }
        
        
      }
    }
    //currentPosEntry.setDouble(getPos());

    
  }
}