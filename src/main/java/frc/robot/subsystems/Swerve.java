// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

//import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
//import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
//import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
//import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
//import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
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
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.util.sendable.SendableBuilder;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
public class Swerve extends SubsystemBase {

  public static enum JoystickConfiguration{
    Controller,
    Joystick,
    RotationalJoystick
  }
  
  //IMU
  Pigeon2 imu = new Pigeon2(Constants.PIGEON_CAN);
  //WPI_Pigeon2 imu = new WPI_Pigeon2(Constants.PIGEON_CAN);

  public Pose2d robotPosition = new Pose2d(new Translation2d(0, 0), new Rotation2d(Math.toRadians(0)));
  GenericEntry joystick_x = Shuffleboard.getTab("Joystick").add("Joystick X-Axis", 0).getEntry();
  GenericEntry joystick_y = Shuffleboard.getTab("Joystick").add("Joystick Y-Axis", 0).getEntry();
  GenericEntry robot_x = Shuffleboard.getTab("Joystick").add("Robot-X-Axis", 0).getEntry();
  GenericEntry robot_y = Shuffleboard.getTab("Joystick").add("Robot-Y-Axis", 0).getEntry();
  // Boolean hi = false;
  // GenericEntry resetRobotPos = Shuffleboard.getTab("Joystick").add("Reset Robot", hi).withWidget(BuiltInWidgets.kToggleButton).getEntry();


  double frontLeftModPos;
  double frontRightModPos;
  double backLeftModPos;
  double backRightModPos;
  
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


  //Show status of each module(velocity, offset, etc)
  ShuffleboardLayout frontLeftModuleStateShuffleboard = Shuffleboard.getTab("Modulestates").getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(0, 0);
  ShuffleboardLayout frontRightModuleStateShuffleboard = Shuffleboard.getTab("Modulestates").getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(2, 0);
  ShuffleboardLayout backLeftModuleStateShuffleboard = Shuffleboard.getTab("Modulestates").getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(4, 0);
  ShuffleboardLayout backRightModuleStateShuffleboard = Shuffleboard.getTab("Modulestates").getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(6, 0);

  Field2d field = new Field2d();
  public static GenericEntry gyroTurn = Shuffleboard.getTab("Joystick").add("Gyro", 0).getEntry();
  GenericEntry currentGyroAngle = Shuffleboard.getTab("Joystick").add("Yaw Angle(Current)", 0).getEntry();
  GenericEntry currentPitch = Shuffleboard.getTab("Joystick").add("Pitch", 0).getEntry();
  GenericEntry currentRoll = Shuffleboard.getTab("Joystick").add("Roll", 0).getEntry();

  ComplexWidget fieldShuffleboard = Shuffleboard.getTab("Field").add("2022-Field", field).withWidget(BuiltInWidgets.kField).withProperties(Map.of("robot icon size", 20));
  //ComplexWidget gyroShufffleboard = Shuffleboard.getTab("Tab Name").add("Gyro", (WPI_Pigeon2)imu);

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


  // public Rotation2d getPosition() {
  //   return Rotation2d.fromDegrees(getTurn)
  // }



  public Swerve() {
    //Shuffleboard.getTab("Joystick").add(imu);
    //imu.configMountPose(AxisDirection.PositiveZ, AxisDirection.PositiveY);
    imu.configMountPose(0, 0, 0);
    //imu.setYaw(0);
    m_odometry.resetPosition(new Rotation2d(0), pos, robotPosition);
    // frontLeftDrivingMotor.setSelectedSensorPosition(0);
    // frontRightDrivingMotor.setSelectedSensorPosition(0);
    // backLeftDrivingMotor.setSelectedSensorPosition(0);
    // backRightDrivingMotor.setSelectedSensorPosition(0);
   
  }
  public void setChasisSpeed(ChassisSpeeds speed){
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

  // public double zAxis(){
  //     return ((RobotContainer.bigdriveStick.getRawAxis(3) * -1) + 1)/2 ;
  // }

  SwerveModulePosition fl = new SwerveModulePosition();
  SwerveModulePosition fr = new SwerveModulePosition();
  SwerveModulePosition bl = new SwerveModulePosition();
  SwerveModulePosition br = new SwerveModulePosition();
  final double dt = 0.02;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_METERS_PER_SECOND);
    
    joystick_x.setDouble(RobotContainer.bigdriveStick.getRawAxis(Constants.X_AXIS));
    joystick_y.setDouble(RobotContainer.bigdriveStick.getRawAxis(Constants.Y_AXIS));
    //SwerveModuleState state = new SwerveModuleState(speedAxis, null);
    // frontLeftModPos = frontLeftDrivingMotor.getSelectedSensorPosition() / (SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI  * 2048) * SdsModuleConfigurations.MK4I_L1.getDriveReduction() / 3.2804;
    // frontRightModPos = frontRightDrivingMotor.getSelectedSensorPosition() / (SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI * 2048) * SdsModuleConfigurations.MK4I_L1.getDriveReduction() / 3.2804;
    // backLeftModPos = backLeftDrivingMotor.getSelectedSensorPosition() / (SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI * 2048) * SdsModuleConfigurations.MK4I_L1.getDriveReduction() / 3.2804;
    // backRightModPos = backRightDrivingMotor.getSelectedSensorPosition() / (SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI * 2048) * SdsModuleConfigurations.MK4I_L1.getDriveReduction() / 3.2804;

    fl.distanceMeters += moduleStates[0].speedMetersPerSecond * dt;
    fr.distanceMeters += moduleStates[1].speedMetersPerSecond * dt;
    bl.distanceMeters += moduleStates[2].speedMetersPerSecond * dt;
    br.distanceMeters += moduleStates[3].speedMetersPerSecond * dt;

    // if(DriverStation.isAutonomous()) {
    //   //speedAxis = 0.3;
    //   speedAxis = 1;
    // } else if(DriverStation.isTeleop()) {
    //   //speedAxis = zAxis();
    //   speedAxis = 1;
    // }

    // SwerveModulePosition[] PPos = {new SwerveModulePosition(0, new Rotation2d(frontLeftModule.getSteerAngle())), 
    //               new SwerveModulePosition(0, new Rotation2d(frontLeftModule.getSteerAngle()))};

    //Need to fix to angles
    frontLeftModule.set(moduleStates[0].speedMetersPerSecond / Constants.MAX_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[0].angle.getRadians());
    frontRightModule.set(moduleStates[1].speedMetersPerSecond / Constants.MAX_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[1].angle.getRadians());
    backLeftModule.set(moduleStates[2].speedMetersPerSecond / Constants.MAX_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[2].angle.getRadians());
    backRightModule.set(moduleStates[3].speedMetersPerSecond / Constants.MAX_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[3].angle.getRadians());

    currentGyroAngle.setDouble(-RobotContainer.m_swerve.gyroAngle().getDegrees());
    currentPitch.setDouble(RobotContainer.m_swerve.pitch().getDegrees());
    currentRoll.setDouble(RobotContainer.m_swerve.roll().getDegrees());
    
    SwerveModulePosition[] modulePositions = {new SwerveModulePosition(fl.distanceMeters , moduleStates[0].angle), 
                                new SwerveModulePosition(fr.distanceMeters , moduleStates[1].angle),
                                new SwerveModulePosition(bl.distanceMeters , moduleStates[2].angle),
                                new SwerveModulePosition(br.distanceMeters, moduleStates[3].angle)};

    SwerveModulePosition[] resetModulePositions = {new SwerveModulePosition(0 , new Rotation2d(0)), 
                                  new SwerveModulePosition(0 , new Rotation2d(0)),
                                  new SwerveModulePosition(0 , new Rotation2d(0)),
                                  new SwerveModulePosition(0, new Rotation2d(0))};

    m_odometry.update(gyroAngle(), modulePositions);

    robotPosition = m_odometry.getPoseMeters();

    field.setRobotPose(robotPosition);
    robot_x.setDouble(m_odometry.getPoseMeters().getX());
    robot_y.setDouble(m_odometry.getPoseMeters().getY());
    //System.out.println("FL: " + moduleStates[0].angle.getDegrees() + "FR: " + moduleStates[1].angle.getDegrees() + "BL: " + moduleStates[2].angle.getDegrees() + "  BR: " + moduleStates[3].angle.getDegrees());

    if(RobotContainer.m_resetRobotPos.getAsBoolean()) {
      m_odometry.resetPosition(new Rotation2d(0), resetModulePositions, new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
      fl.distanceMeters = 0;
      fr.distanceMeters = 0;
      bl.distanceMeters = 0;
      br.distanceMeters = 0;
    }
    //System.out.println(SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI);
  }
} 