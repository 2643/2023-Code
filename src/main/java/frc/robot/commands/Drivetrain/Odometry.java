// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

// import java.util.ArrayList;
// import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
import frc.robot.RobotContainer;
// import frc.robot.subsystems.ArmGrab.States;

public class Odometry extends CommandBase {
    Pose2d pos;
    Pose2d targetPos;
    DoubleSupplier x_vel;
    DoubleSupplier y_vel;
    DoubleSupplier turn_vel;
    double targetTurnDegrees;
    boolean isPickup;

    double kP = 1;
    double kI = 0.5;
    double kD = 0;

    double rotational_kP = 0.04;
    double rotational_kI = 0;
    double rotational_kD = 0;

    TrapezoidProfile.Constraints constraints = new Constraints(3/10, 2/10);
    TrapezoidProfile.Constraints rotation_constraints = new TrapezoidProfile.Constraints(500, 1000000000);

    //TrapezoidProfile.Constraints rotation_constraints = new Constraints(10/100, 10/10);

    ProfiledPIDController xPIDController = new ProfiledPIDController(kP, kI, kD, constraints);
    ProfiledPIDController yPIDController = new ProfiledPIDController(kP, kI, kD, constraints);
    ProfiledPIDController rotationPIDController = new ProfiledPIDController(rotational_kP, rotational_kI, rotational_kD, rotation_constraints);

    
    //GenericEntry kDE = Shuffleboard.getTab("Swerve").add("kDE", 0).getEntry();
  /** Creates a new SwerveDriveOdometry. */
  //Used for Automation
  
  public Odometry(Pose2d m_targetPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
    addRequirements(RobotContainer.m_vision);
    targetPos = m_targetPos;
  }

  // public Odometry(Pose2d m_targetPos, boolean isPickup) {
  //   // Use addRequirements() here to declare subsystem dependencies.
  //   addRequirements(RobotContainer.m_drivetrain);
  //   addRequirements(RobotContainer.m_vision);
  //   targetPos = m_targetPos;
  //   //this.isPickup = isPickup;
  // }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xPIDController.setTolerance(0.02);
    yPIDController.setTolerance(0.02);
    rotationPIDController.enableContinuousInput(0, 360);
    //rotationPIDController.setTolerance(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pos = RobotContainer.m_drivetrain.getPose();
    //targetPos = new Pose2d(Drivetrain.targetX.getDouble(0), Drivetrain.targetY.getDouble(0), new Rotation2d());

    // xPIDController.setPID(Drivetrain.kPE.getDouble(0), Drivetrain.kIE.getDouble(0), Drivetrain.kDE.getDouble(0));
    // yPIDController.setPID(Drivetrain.kPE.getDouble(0), Drivetrain.kIE.getDouble(0), Drivetrain.kDE.getDouble(0));

    x_vel = () -> xPIDController.calculate(pos.getX(), targetPos.getX());
    if(xPIDController.atGoal()) {
      x_vel = () -> 0;
    }
    y_vel = () -> yPIDController.calculate(pos.getY(), targetPos.getY());
    if(yPIDController.atGoal()) {
      y_vel = () -> 0;
    }
    turn_vel = () -> rotationPIDController.calculate(RobotContainer.m_drivetrain.gyroAngle().getDegrees(), targetPos.getRotation().getDegrees());
    if(rotationPIDController.atGoal()) {
      turn_vel = () -> 0;
    }

    //TODO: CHANGE TO OMEGA RADIANS PER SECOND AFTER MOVING WORKS
    RobotContainer.m_drivetrain.setChassisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(-x_vel.getAsDouble(), -y_vel.getAsDouble(),
        -turn_vel.getAsDouble(), RobotContainer.m_drivetrain.gyroAngle()));
    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_drivetrain.setChassisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
        0, RobotContainer.m_drivetrain.gyroAngle()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((Math.round(pos.getX() * 5) == Math.round(targetPos.getX() * 5)) 
      && ((Math.round(pos.getY() * 5) == Math.round(targetPos.getY() * 5))) 
      && (Math.round(RobotContainer.m_drivetrain.gyroAngle().getDegrees()) == Math.round(targetPos.getRotation().getDegrees()))) { //&& Math.round(targetTurnDegrees / 5) == Math.round(RobotContainer.m_drivetrain.gyroAngle().getDegrees() / 5)) {
        return true;
    }
    return false;
  }
}
