// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveDrive extends CommandBase {
  double encoderkP = 0.2;
  double encoderkI = 0;
  double encoderkD = 0;

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1000, 1000000000);
  ProfiledPIDController rotational_pid = new ProfiledPIDController(encoderkP, encoderkI, encoderkD, constraints);
  
  static DoubleSupplier xAxisValue;
  static DoubleSupplier yAxisValue;
  static DoubleSupplier rotationalXAxisValue;
  double current;
  double betterEncoderAngle;

  
 
  
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
    rotational_pid.enableContinuousInput(0, 360);
    
    // encoder_pid = new PIDController(encoderkP, encoderkI, encoderkD);
    // encoder_pid.calculate(RobotContainer.m_drivetrain.gyroAngle().getDegrees(), encoderAngle);
  }

  private static double deadbandCalc(double joystickRawAxis, double deadband) {
    if (Math.abs(joystickRawAxis) > deadband) {
      if (joystickRawAxis > deadband) {
        return (joystickRawAxis - deadband) / (1.0 - deadband);
      } else {
        return (joystickRawAxis + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double squareAxis(double value, double deadband) {
    value = deadbandCalc(value, deadband);
    value = Math.copySign(value * value, value);
    return value;
  }

  private static double logAxis(double value) {
    value = Math.copySign(Math.log((Math.abs(value) + 1)) / Math.log(2), value);
    return value;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //rotational_pid.setPID(Constants.kP.getDouble(0), Constants.kI.getDouble(0), Constants.kD.getDouble(0));

    betterEncoderAngle = (RobotContainer.operatorBoard.getRawAxis(3) + 1) * 180;
    // Constants.targetEncoderAngle.setDouble(betterEncoderAngle);
    // Constants.currentAngle.setDouble(RobotContainer.m_drivetrain.gyroAngle().getDegrees());

    xAxisValue = () -> -squareAxis(logAxis(RobotContainer.swerveStick.getRawAxis(Constants.X_AXIS_PORT)), 0.05) * Constants.MAX_METERS_PER_SECOND;
    yAxisValue = () -> -squareAxis(logAxis(RobotContainer.swerveStick.getRawAxis(Constants.Y_AXIS_PORT)), 0.05) * Constants.MAX_METERS_PER_SECOND;

    if(RobotContainer.turningMode.getAsBoolean()) {
      rotationalXAxisValue = () -> squareAxis(logAxis(RobotContainer.swerveStick.getRawAxis(Constants.ROTATIONAL_AXIS_PORT)), 0.3) * Constants.MAX_RADIANS_PER_SECOND;
    } else {
      rotationalXAxisValue = () -> -rotational_pid.calculate(RobotContainer.m_drivetrain.gyroAngle().getDegrees()+90, betterEncoderAngle);
    }
    if(!RobotContainer.autoBottom.getAsBoolean() && !RobotContainer.autoMiddle.getAsBoolean()) {
      RobotContainer.m_drivetrain.setChassisSpeed(!RobotContainer.m_robotRelativeMode.getAsBoolean() ? ChassisSpeeds.fromFieldRelativeSpeeds(yAxisValue.getAsDouble(), xAxisValue.getAsDouble(), rotationalXAxisValue.getAsDouble(), RobotContainer.m_drivetrain.gyroAngle()) : new ChassisSpeeds(xAxisValue.getAsDouble(), -yAxisValue.getAsDouble(), rotationalXAxisValue.getAsDouble()));
    } else {
      RobotContainer.m_drivetrain.setChassisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, RobotContainer.m_drivetrain.gyroAngle()));}
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_drivetrain.setChassisSpeed(Constants.FIELD_RELATIVE_MODE ? ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, -0, RobotContainer.m_drivetrain.gyroAngle()) : new ChassisSpeeds(0, 0, 0));  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
