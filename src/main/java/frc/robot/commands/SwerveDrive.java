// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class SwerveDrive extends CommandBase {
  static DoubleSupplier xAxisValue;
  static DoubleSupplier yAxisValue;
  static DoubleSupplier rotationalXAxisValue;
  public static Swerve.JoystickConfiguration m_joystick;
  
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerve);
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
    // if(Constants.M_JOYSTICK == Swerve.JoystickConfiguration.Joystick) {
    //   xAxisValue = () -> -squareAxis(RobotContainer.bigdriveStick.getRawAxis(0)) * Constants.MAX_METERS_PER_SECOND;
    //   yAxisValue = () -> -squareAxis(RobotContainer.bigdriveStick.getRawAxis(1)) * -Constants.MAX_METERS_PER_SECOND;
    //   rotationalXAxisValue = () -> -squareAxis(RobotContainer.bigdriveStick.getRawAxis(Constants.ROTATIONAL_AXIS)) * Constants.MAX_RADIANS_PER_SECOND;
    // } else if(Constants.M_JOYSTICK == Swerve.JoystickConfiguration.Controller) {
    
    //   xAxisValue = () -> -squareAxis(RobotContainer.driveStick.getRawAxis(Constants.X_AXIS)) * Constants.MAX_METERS_PER_SECOND;
    //   yAxisValue = () -> -squareAxis(RobotContainer.driveStick.getRawAxis(Constants.Y_AXIS)) * -Constants.MAX_METERS_PER_SECOND;
    //   rotationalXAxisValue = () -> -squareAxis(RobotContainer.bigdriveStick.getRawAxis(Constants.ROTATIONAL_AXIS)) * Constants.MAX_RADIANS_PER_SECOND;

    // } else if(Constants.M_JOYSTICK == Swerve.JoystickConfiguration.RotationalJoystick) {

    xAxisValue = () -> -squareAxis(logAxis(RobotContainer.bigdriveStick.getRawAxis(Constants.X_AXIS)), 0.02) * Constants.MAX_METERS_PER_SECOND;
    yAxisValue = () -> -squareAxis(logAxis(RobotContainer.bigdriveStick.getRawAxis(Constants.Y_AXIS)), 0.02) * -Constants.MAX_METERS_PER_SECOND;
    rotationalXAxisValue = () -> -logAxis(squareAxis(RobotContainer.bigdriveStick.getRawAxis(Constants.ROTATIONAL_AXIS), 0.75)) * -Constants.MAX_RADIANS_PER_SECOND; //* Swerve.speedAxis / 2;
    if(Math.round(Swerve.gyroTurn.getDouble(0)/5) == Math.round(RobotContainer.m_swerve.gyroAngle().getDegrees()/5))
      rotationalXAxisValue = () -> 0;
    else if(Math.abs(Swerve.gyroTurn.getDouble(0) - (RobotContainer.m_swerve.gyroAngle().getDegrees())) < 15)
      rotationalXAxisValue = () -> 0.1 * Constants.MAX_RADIANS_PER_SECOND * Math.copySign(1, Swerve.gyroTurn.getDouble(0) - (RobotContainer.m_swerve.gyroAngle().getDegrees()));
    else if(Math.abs(Swerve.gyroTurn.getDouble(0) - (RobotContainer.m_swerve.gyroAngle().getDegrees())) < 50)
      rotationalXAxisValue = () -> 0.3 * Constants.MAX_RADIANS_PER_SECOND * Math.copySign(1, Swerve.gyroTurn.getDouble(0) - (RobotContainer.m_swerve.gyroAngle().getDegrees()));
    else if(Swerve.gyroTurn.getDouble(0) > RobotContainer.m_swerve.gyroAngle().getDegrees())
      rotationalXAxisValue = () -> 0.5 * Constants.MAX_RADIANS_PER_SECOND;
    else if(Swerve.gyroTurn.getDouble(0) < RobotContainer.m_swerve.gyroAngle().getDegrees())
      rotationalXAxisValue = () -> -0.5 * Constants.MAX_RADIANS_PER_SECOND;
    
    
    RobotContainer.m_swerve.setChasisSpeed(Constants.FIELD_RELATIVE_MODE ? ChassisSpeeds.fromFieldRelativeSpeeds(xAxisValue.getAsDouble(), yAxisValue.getAsDouble(),
    -rotationalXAxisValue.getAsDouble(), RobotContainer.m_swerve.gyroAngle()) : new ChassisSpeeds(xAxisValue.getAsDouble(), yAxisValue.getAsDouble(), rotationalXAxisValue.getAsDouble()));
    
    //RobotContainer.m_swerve.setChasisSpeed(new ChassisSpeeds(xAxisValue.getAsDouble(), yAxisValue.getAsDouble(), -rotationalXAxisValue.getAsDouble()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_swerve.setChasisSpeed(Constants.FIELD_RELATIVE_MODE ? ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
    -0, RobotContainer.m_swerve.gyroAngle()) : new ChassisSpeeds(0, 0, 0));
    RobotContainer.m_swerve.setChasisSpeed(new ChassisSpeeds(0, 0, -0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
