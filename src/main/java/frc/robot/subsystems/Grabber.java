// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */
  static CANSparkMax grabberMotor1 = new CANSparkMax(Constants.ArmGrab.GRABBER_PORT_ONE, MotorType.kBrushless);
  static CANSparkMax grabberMotor2 = new CANSparkMax(Constants.ArmGrab.GRABBER_PORT_TWO, MotorType.kBrushless);

  double outputCurrent = 0;
  GenericEntry OutputCurrentEntry = Shuffleboard.getTab("Grabber").add("Output Current", outputCurrent).getEntry();
  static Timer timer = new Timer(); 

  public Grabber() {
    grabberMotor2.follow(grabberMotor1, true);
    grabberMotor1.setIdleMode(IdleMode.kBrake);
  }

  public enum States{
    NOTHING,
    PULL_START,
    PULL_TIME_ELAPSING,
    PULLING,
    PUSH_START,
    PUSHING_TIME_ELAPSING,
    PUSHING,
    STOP_MOTOR_PULL,
    STOP_MOTOR_PUSH;

  }


  public static States state = States.NOTHING;
  GenericEntry StateEntry = Shuffleboard.getTab("Grabber").add("State", "Nothing").getEntry();

  public static void percentOutput(double output){
    grabberMotor1.getPIDController().setReference(output, ControlType.kDutyCycle);
  }
  // public static void percentOutputGrabber2(double output){
  //   grabberMotor2.getPIDController().setReference(output, ControlType.kDutyCycle);
  // }
  public static double getOutputCurrentGrabber1(){
    return grabberMotor1.getOutputCurrent();
  }

  public static void stopMotor(){
    grabberMotor1.getPIDController().setReference(0, ControlType.kDutyCycle);
  }


  public static void firstCurrentPassPulling(){
    timer.start();
    if(timer.hasElapsed(0.5)){
      timer.stop();
      timer.reset();
      state = States.PULLING;

      // state = States.CLOSING_CURRENT;
    }
  }

  public static void firstCurrentPassPushing(){
    timer.start();
    if(timer.hasElapsed(0.5)){
      timer.stop();
      timer.reset();
      state = States.PUSHING;

      // state = States.CLOSING_CURRENT;
    }
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
   
    if(DriverStation.isEnabled()) {
      outputCurrent = getOutputCurrentGrabber1();
      OutputCurrentEntry.setDouble(outputCurrent);
      switch(state) {
        case NOTHING:
          StateEntry.setString("Nothing");
          break;
        case PULL_START:
          StateEntry.setString("Pull Start");
          percentOutput(Constants.ArmGrab.GRABBER_MOVE_GAME_PIECE_SPEED);
          state = States.PULL_TIME_ELAPSING;
          break;
        case PULL_TIME_ELAPSING:
          StateEntry.setString("Pull Time Elapsing");
          firstCurrentPassPulling();
          break;
        case PULLING:
          StateEntry.setString("Pulling");
          if (RobotContainer.coneMode.getAsBoolean()){
            if (Constants.ArmGrab.CONE_OUTPUT_CURRENT_MAX <= getOutputCurrentGrabber1()) {
              state = States.STOP_MOTOR_PULL;
              
            }
          }
          else{
            if (Constants.ArmGrab.CUBE_OUTPUT_CURRENT_MAX <=  getOutputCurrentGrabber1()) {
              state = States.STOP_MOTOR_PUSH;
            }
          }
          break;

        // case PULLED:
        //   System.out.println("Pulled");

        //   StateEntry.setDouble(4);
        //   grabberMotor1.setIdleMode(IdleMode.kBrake);

        //   stopMotor();
        //   //percentOutput(Constants.GRABBBER_CONSTANT_PERCENT_OUTPUT);
        //   break;
        case PUSH_START:
          StateEntry.setString("Push Start");
          percentOutput(-Constants.ArmGrab.GRABBER_MOVE_GAME_PIECE_SPEED);
          state = States.PUSHING_TIME_ELAPSING;
          break;
        case PUSHING_TIME_ELAPSING:
          StateEntry.setString("Pushing Time Elapsing");
          firstCurrentPassPushing();
         break;
        case PUSHING:
          StateEntry.setString("Pushing");
          if (Constants.ArmGrab.GRABBER_EMPTY_OUTPUT_MAX  <= getOutputCurrentGrabber1() ){
            state = States.STOP_MOTOR_PUSH;
          }
          break;
        // case PUSHED:
        //   System.out.println("Pushed");

        //   StateEntry.setDouble(8);
        //   grabberMotor1.setIdleMode(IdleMode.kBrake);
        //   stopMotor();
        //   break
        
        case STOP_MOTOR_PULL:
          StateEntry.setString("Stop Motor");
          stopMotor();
          break;
        case STOP_MOTOR_PUSH:
          StateEntry.setString("Stop Motor");
          stopMotor();
          break;
        default:
          StateEntry.setDouble(9);
          System.out.print("something went wrong");
          break;
          
      }
  }
 }
}