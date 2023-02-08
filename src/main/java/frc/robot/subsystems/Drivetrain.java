// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {

  public static enum JoystickConfiguration{
    Controller,
    Joystick,
    RotationalJoystick
  }
  
  Pigeon2 imu = new Pigeon2(Constants.PIGEON_CAN);

  public Pose2d robotPosition = new Pose2d(new Translation2d(0, 0), new Rotation2d(Math.toRadians(0)));
  GenericEntry joystick_x = Shuffleboard.getTab("Joystick").add("Joystick X-Axis", 0).getEntry();
  GenericEntry joystick_y = Shuffleboard.getTab("Joystick").add("Joystick Y-Axis", 0).getEntry();
  GenericEntry robot_x = Shuffleboard.getTab("Joystick").add("Robot-X-Axis", 0).getEntry();
  GenericEntry robot_y = Shuffleboard.getTab("Joystick").add("Robot-Y-Axis", 0).getEntry();

  double frontLeftModPos;
  double frontRightModPos;
  double backLeftModPos;
  double backRightModPos;

  SwerveModulePosition fl = new SwerveModulePosition();
  SwerveModulePosition fr = new SwerveModulePosition();
  SwerveModulePosition bl = new SwerveModulePosition();
  SwerveModulePosition br = new SwerveModulePosition();
  
  final double dt = 0.02;
  
  //public static double speedAxis;
  SwerveModulePosition[] currentPos;

  //Module placement from the center of robot
  Translation2d m_frontLeftLocation = new Translation2d(Constants.TRANSLATION_2D_METERS, Constants.TRANSLATION_2D_METERS);
  Translation2d m_frontRightLocation = new Translation2d(Constants.TRANSLATION_2D_METERS, -Constants.TRANSLATION_2D_METERS);
  Translation2d m_backLeftLocation = new Translation2d(-Constants.TRANSLATION_2D_METERS, Constants.TRANSLATION_2D_METERS);
  Translation2d m_backRightLocation = new Translation2d(-Constants.TRANSLATION_2D_METERS, -Constants.TRANSLATION_2D_METERS);

  //Kinematics and Odometry for Swerve Drive
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  SwerveModulePosition[] pos = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};

  public SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, gyroAngle(), pos);
  Field2d field = new Field2d();

  //Show status of each module(velocity, offset, etc)
  ShuffleboardTab moduleStateTab = Shuffleboard.getTab("Modulestates");
  ShuffleboardLayout frontLeftModuleStateShuffleboard = moduleStateTab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(0, 0);
  ShuffleboardLayout frontRightModuleStateShuffleboard = moduleStateTab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(2, 0);
  ShuffleboardLayout backLeftModuleStateShuffleboard = moduleStateTab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(4, 0);
  ShuffleboardLayout backRightModuleStateShuffleboard = moduleStateTab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(6, 0);

  static ShuffleboardTab infoTab = Shuffleboard.getTab("Info");
  public static GenericEntry gyroTurn = infoTab.add("Target Gyro Yaw", 0).getEntry();
  GenericEntry currentGyroAngle = infoTab.add("Current Yaw", 0).getEntry();
  GenericEntry currentPitch = infoTab.add("Pitch", 0).getEntry();
  GenericEntry currentRoll = infoTab.add("Roll", 0).getEntry();

  ComplexWidget fieldShuffleboard = Shuffleboard.getTab("Field").add("2023-Field", field).withWidget(BuiltInWidgets.kField).withProperties(Map.of("robot icon size", 20));

  //Starting ChassisSpeed
  ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

  GearRatio gearRatio = Mk4iSwerveModuleHelper.GearRatio.L1;
  //Each module created
  SwerveModule frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(frontLeftModuleStateShuffleboard, gearRatio, Constants.FRONT_LEFT_DRIVE_MOTOR, Constants.FRONT_LEFT_TURN_MOTOR, Constants.FRONT_LEFT_CANCODER, Constants.FRONT_LEFT_TURN_OFFSET);
  SwerveModule frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(frontRightModuleStateShuffleboard, gearRatio, Constants.FRONT_RIGHT_DRIVE_MOTOR, Constants.FRONT_RIGHT_TURN_MOTOR, Constants.FRONT_RIGHT_CANCODER, Constants.FRONT_RIGHT_TURN_OFFSET);
  SwerveModule backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(backLeftModuleStateShuffleboard, gearRatio, Constants.REAR_LEFT_DRIVE_MOTOR, Constants.REAR_LEFT_TURN_MOTOR, Constants.REAR_LEFT_CANCODER, Constants.REAR_LEFT_TURN_OFFSET);
  SwerveModule backRightModule = Mk4iSwerveModuleHelper.createFalcon500(backRightModuleStateShuffleboard, gearRatio, Constants.REAR_RIGHT_DRIVE_MOTOR, Constants.REAR_RIGHT_TURN_MOTOR, Constants.REAR_RIGHT_CANCODER, Constants.REAR_RIGHT_TURN_OFFSET);

  // TalonFX frontLeftDrivingMotor = new TalonFX(Constants.FRONT_LEFT_DRIVE_MOTOR);
  // TalonFX frontRightDrivingMotor = new TalonFX(Constants.FRONT_RIGHT_DRIVE_MOTOR);
  // TalonFX backLeftDrivingMotor = new TalonFX(Constants.REAR_LEFT_DRIVE_MOTOR);
  // TalonFX backRightDrivingMotor = new TalonFX(Constants.REAR_RIGHT_DRIVE_MOTOR);

  // CANCoder fLCanCoder = new CANCoder(Constants.FRONT_LEFT_CANCODER);
  // CANCoder fRCanCoder = new CANCoder(Constants.FRONT_RIGHT_CANCODER);
  // CANCoder bLCanCoder = new CANCoder(Constants.REAR_LEFT_CANCODER);
  // CANCoder bRCanCoder = new CANCoder(Constants.REAR_RIGHT_CANCODER);

  public Drivetrain() {
    imu.configFactoryDefault();
    imu.setYaw(0);
    m_odometry.resetPosition(new Rotation2d(0), pos, robotPosition);
   
  }
  public void setChassisSpeed(ChassisSpeeds speed){
    speeds = speed;
  }

  public Rotation2d gyroAngle() {
    return Rotation2d.fromDegrees(imu.getYaw());
  }

  public Rotation2d pitch() {
    return Rotation2d.fromDegrees(imu.getPitch());
  }

  public Rotation2d roll() {
    return Rotation2d.fromDegrees(imu.getRoll());
  }
  
  public void resetGyro(){
    imu.setYaw(0);
  }
  SwerveModulePosition[] resetModulePosition = {new SwerveModulePosition(0, new Rotation2d(0)), 
    new SwerveModulePosition(0, new Rotation2d(0)),
    new SwerveModulePosition(0, new Rotation2d(0)),
    new SwerveModulePosition(0, new Rotation2d(0))};

  public void resetOdometry() {
      m_odometry.resetPosition(new Rotation2d(0), resetModulePosition, new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
      fl.distanceMeters = 0;
      fr.distanceMeters = 0;
      bl.distanceMeters = 0;
      br.distanceMeters = 0;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_METERS_PER_SECOND);
    
    joystick_x.setDouble(RobotContainer.swerveStick.getRawAxis(Constants.X_AXIS_PORT));
    joystick_y.setDouble(RobotContainer.swerveStick.getRawAxis(Constants.Y_AXIS_PORT));

    // frontLeftModPos = frontLeftDrivingMotor.getSelectedSensorPosition() / (SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI  * 2048) * SdsModuleConfigurations.MK4I_L1.getDriveReduction() / 3.2804;
    // frontRightModPos = frontRightDrivingMotor.getSelectedSensorPosition() / (SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI * 2048) * SdsModuleConfigurations.MK4I_L1.getDriveReduction() / 3.2804;
    // backLeftModPos = backLeftDrivingMotor.getSelectedSensorPosition() / (SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI * 2048) * SdsModuleConfigurations.MK4I_L1.getDriveReduction() / 3.2804;
    // backRightModPos = backRightDrivingMotor.getSelectedSensorPosition() / (SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI * 2048) * SdsModuleConfigurations.MK4I_L1.getDriveReduction() / 3.2804;

    fl.distanceMeters += moduleStates[0].speedMetersPerSecond * dt;
    fr.distanceMeters += moduleStates[1].speedMetersPerSecond * dt;
    bl.distanceMeters += moduleStates[2].speedMetersPerSecond * dt;
    br.distanceMeters += moduleStates[3].speedMetersPerSecond * dt;

    //Need to fix to angles
    frontLeftModule.set(moduleStates[0].speedMetersPerSecond / Constants.MAX_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[0].angle.getRadians());
    frontRightModule.set(moduleStates[1].speedMetersPerSecond / Constants.MAX_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[1].angle.getRadians());
    backLeftModule.set(moduleStates[2].speedMetersPerSecond / Constants.MAX_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[2].angle.getRadians());
    backRightModule.set(moduleStates[3].speedMetersPerSecond / Constants.MAX_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[3].angle.getRadians());

    currentGyroAngle.setDouble(-gyroAngle().getDegrees());
    currentPitch.setDouble(pitch().getDegrees());
    currentRoll.setDouble(roll().getDegrees());
    
    SwerveModulePosition[] modulePositions = {new SwerveModulePosition(fl.distanceMeters , moduleStates[0].angle), 
                                              new SwerveModulePosition(fr.distanceMeters, moduleStates[1].angle),
                                              new SwerveModulePosition(bl.distanceMeters, moduleStates[2].angle),
                                              new SwerveModulePosition(br.distanceMeters, moduleStates[3].angle)};

    m_odometry.update(gyroAngle(), modulePositions);
    robotPosition = m_odometry.getPoseMeters();
    if(RobotContainer.m_resetRobotPos.getAsBoolean()){
      resetOdometry();
    }
    field.setRobotPose(robotPosition);
    robot_x.setDouble(m_odometry.getPoseMeters().getX());
    robot_y.setDouble(m_odometry.getPoseMeters().getY());
  }
} 