// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {
  Pigeon2 imu = new Pigeon2(Constants.PIGEON_CAN);

  public Pose2d robotPosition = new Pose2d(new Translation2d(0, 0), new Rotation2d(Math.toRadians(0)));
  // GenericEntry joystick_x = Shuffleboard.getTab("Joystick").add("Joystick X-Axis", 0).getEntry();
  // GenericEntry joystick_y = Shuffleboard.getTab("Joystick").add("Joystick Y-Axis", 0).getEntry();
  
  // public static double speedAxis;
  SwerveModulePosition[] currentPos;

  // Module placement from the center of robot
  static Translation2d m_frontLeftLocation = new Translation2d(Constants.TRANSLATION_2D_METERS, Constants.TRANSLATION_2D_METERS);
  static Translation2d m_frontRightLocation = new Translation2d(Constants.TRANSLATION_2D_METERS, -Constants.TRANSLATION_2D_METERS);
  static Translation2d m_backLeftLocation = new Translation2d(-Constants.TRANSLATION_2D_METERS, Constants.TRANSLATION_2D_METERS);
  static Translation2d m_backRightLocation = new Translation2d(-Constants.TRANSLATION_2D_METERS, -Constants.TRANSLATION_2D_METERS);

  // Kinematics and Odometry for Swerve Drive
  public static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  SwerveModulePosition[] pos = { new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() };

  public SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, gyroAngle(), pos);
  Field2d field = new Field2d();

  // Show status of each module(velocity, offset, etc)
  ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
  ShuffleboardLayout frontLeftModuleStateShuffleboard = debugTab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(0, 0);
  ShuffleboardLayout frontRightModuleStateShuffleboard = debugTab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(2, 0);
  ShuffleboardLayout backLeftModuleStateShuffleboard = debugTab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(4, 0);
  ShuffleboardLayout backRightModuleStateShuffleboard = debugTab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(6, 0);
  //GenericEntry robot_x = Shuffleboard.getTab("Debug").add("Swerve-X-Axis", 0).getEntry();

  ShuffleboardLayout drivetrainLayout = debugTab.getLayout("Drivetrain", BuiltInLayouts.kGrid).withSize(4, 2).withPosition(8, 0);
  GenericEntry robot_y = drivetrainLayout.add("Swerve-Y-Axis", 0).withPosition(1, 0).getEntry();
  GenericEntry robot_x = drivetrainLayout.add("Swerve-X-Axis", 0).withPosition(0, 0).getEntry();


  //static ShuffleboardTab infoTab = Shuffleboard.getTab("Debug");
  //public GenericEntry gyroTurn = drivetrainLayout.add("Target Gyro Yaw", 0).getEntry();
  GenericEntry currentGyroAngle = drivetrainLayout.add("Current Yaw", 0).withPosition(2, 0).getEntry();
  // GenericEntry currentPitch = infoTab.add("Pitch", 0).getEntry();
  // GenericEntry currentRoll = infoTab.add("Roll", 0).getEntry();

  //ComplexWidget fieldShuffleboard = Shuffleboard.getTab("Field").add("2023-Field", field)
  //    .withWidget(BuiltInWidgets.kField).withProperties(Map.of("robot icon size", 20));
  public SwerveDrivePoseEstimator poseVision = new SwerveDrivePoseEstimator(m_kinematics, gyroAngle(), pos, new Pose2d());

  // Starting ChassisSpeed
  ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
  SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

  // GearRatio gearRatio = Mk4iSwerveModuleHelper.GearRatio.L1;

  // TalonFX frontLeftDrivingMotor = new
  // TalonFX(Constants.FRONT_LEFT_DRIVE_MOTOR);
  // TalonFX frontRightDrivingMotor = new
  // TalonFX(Constants.FRONT_RIGHT_DRIVE_MOTOR);
  // TalonFX backLeftDrivingMotor = new TalonFX(Constants.REAR_LEFT_DRIVE_MOTOR);
  // TalonFX backRightDrivingMotor = new
  // TalonFX(Constants.REAR_RIGHT_DRIVE_MOTOR);

  // TalonFX frontLeftTurningMotor = new
  // TalonFX(Constants.FRONT_LEFT_DRIVE_MOTOR);
  // TalonFX frontRightTurningMotor = new
  // TalonFX(Constants.FRONT_RIGHT_DRIVE_MOTOR);
  // TalonFX backLeftTurningMotor = new TalonFX(Constants.REAR_LEFT_DRIVE_MOTOR);
  // TalonFX backRightTurningMotor = new
  // TalonFX(Constants.REAR_RIGHT_DRIVE_MOTOR);

  // SwerveModule frontLeftModule =
  // Mk4iSwerveModuleHelper.createFalcon500(frontLeftModuleStateShuffleboard,
  // gearRatio, Constants.FRONT_LEFT_DRIVE_MOTOR, Constants.FRONT_LEFT_TURN_MOTOR,
  // Constants.FRONT_LEFT_CANCODER, Constants.FRONT_LEFT_TURN_OFFSET);
  // SwerveModule frontRightModule =
  // Mk4iSwerveModuleHelper.createFalcon500(frontRightModuleStateShuffleboard,
  // gearRatio, Constants.FRONT_RIGHT_DRIVE_MOTOR,
  // Constants.FRONT_RIGHT_TURN_MOTOR, Constants.FRONT_RIGHT_CANCODER,
  // Constants.FRONT_RIGHT_TURN_OFFSET);
  // SwerveModule backLeftModule =
  // Mk4iSwerveModuleHelper.createFalcon500(backLeftModuleStateShuffleboard,
  // gearRatio, Constants.REAR_LEFT_DRIVE_MOTOR, Constants.REAR_LEFT_TURN_MOTOR,
  // Constants.REAR_LEFT_CANCODER, Constants.REAR_LEFT_TURN_OFFSET);
  // SwerveModule backRightModule =
  // Mk4iSwerveModuleHelper.createFalcon500(backRightModuleStateShuffleboard,
  // gearRatio, Constants.REAR_RIGHT_DRIVE_MOTOR, Constants.REAR_RIGHT_TURN_MOTOR,
  // Constants.REAR_RIGHT_CANCODER, Constants.REAR_RIGHT_TURN_OFFSET);

  // GearRatio gearRatio = Mk4iSwerveModuleHelper.GearRatio.L1;
  // Each module created
  SwerveModule frontLeftModule = new MkSwerveModuleBuilder().withLayout(frontLeftModuleStateShuffleboard)
      .withGearRatio(SdsModuleConfigurations.MK4I_L1).withDriveMotor(MotorType.FALCON, Constants.FRONT_LEFT_DRIVE_MOTOR)
      .withSteerMotor(MotorType.FALCON, Constants.FRONT_LEFT_TURN_MOTOR)
      .withSteerEncoderPort(Constants.FRONT_LEFT_CANCODER).withSteerOffset(Constants.FRONT_LEFT_TURN_OFFSET).build();
  SwerveModule frontRightModule = new MkSwerveModuleBuilder().withLayout(frontRightModuleStateShuffleboard)
      .withGearRatio(SdsModuleConfigurations.MK4I_L1)
      .withDriveMotor(MotorType.FALCON, Constants.FRONT_RIGHT_DRIVE_MOTOR)
      .withSteerMotor(MotorType.FALCON, Constants.FRONT_RIGHT_TURN_MOTOR)
      .withSteerEncoderPort(Constants.FRONT_RIGHT_CANCODER).withSteerOffset(Constants.FRONT_RIGHT_TURN_OFFSET).build();
  SwerveModule backLeftModule = new MkSwerveModuleBuilder().withLayout(backLeftModuleStateShuffleboard)
      .withGearRatio(SdsModuleConfigurations.MK4I_L1).withDriveMotor(MotorType.FALCON, Constants.REAR_LEFT_DRIVE_MOTOR)
      .withSteerMotor(MotorType.FALCON, Constants.REAR_LEFT_TURN_MOTOR)
      .withSteerEncoderPort(Constants.REAR_LEFT_CANCODER).withSteerOffset(Constants.REAR_LEFT_TURN_OFFSET).build();
  SwerveModule backRightModule = new MkSwerveModuleBuilder().withLayout(backRightModuleStateShuffleboard)
      .withGearRatio(SdsModuleConfigurations.MK4I_L1).withDriveMotor(MotorType.FALCON, Constants.REAR_RIGHT_DRIVE_MOTOR)
      .withSteerMotor(MotorType.FALCON, Constants.REAR_RIGHT_TURN_MOTOR)
      .withSteerEncoderPort(Constants.REAR_RIGHT_CANCODER).withSteerOffset(Constants.REAR_RIGHT_TURN_OFFSET).build();

  // public static GenericEntry kPE = Shuffleboard.getTab("SwerveE").add("kPE", 0.2).getEntry();
  // public static GenericEntry kIE = Shuffleboard.getTab("SwerveE").add("kIE", 0).getEntry();
  // public static GenericEntry kDE = Shuffleboard.getTab("SwerveE").add("kDE", 0).getEntry();

  // public static GenericEntry targetX = Shuffleboard.getTab("SwerveE").add("targetX", 0.2).getEntry();
  // public static GenericEntry targetY = Shuffleboard.getTab("SwerveE").add("targetY", 0).getEntry();

  public Drivetrain() {
    // frontLeftTurningMotor.configFactoryDefault();
    // frontRightTurningMotor.configFactoryDefault();
    // backLeftTurningMotor.configFactoryDefault();
    // backRightTurningMotor.configFactoryDefault();
    imu.configFactoryDefault();
    imu.setYaw(0);
    m_odometry.resetPosition(new Rotation2d(0), pos, robotPosition);
    // Timer.delay(1.0);

  }

  public void setChassisSpeed(ChassisSpeeds speed) {
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

  public void resetGyro() {
    imu.setYaw(0);
  }

  public void setYaw(double yaw) {
    imu.setYaw(yaw);
  }

  SwerveModulePosition[] resetModulePosition = { new SwerveModulePosition(0, new Rotation2d(0)),
      new SwerveModulePosition(0, new Rotation2d(0)),
      new SwerveModulePosition(0, new Rotation2d(0)),
      new SwerveModulePosition(0, new Rotation2d(0)) };

  public void setModuleStates(SwerveModuleState[] desiREDStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiREDStates, Constants.AUTONOMOUS_VELOCITY_PER_SECOND);

    frontLeftModule.set(desiREDStates[1].speedMetersPerSecond / Constants.MAX_METERS_PER_SECOND * Constants.MAX_VOLTAGE,
        moduleStates[0].angle.getRadians());
    frontRightModule.set(
        desiREDStates[2].speedMetersPerSecond / Constants.MAX_METERS_PER_SECOND * Constants.MAX_VOLTAGE,
        moduleStates[1].angle.getRadians());
    backLeftModule.set(desiREDStates[3].speedMetersPerSecond / Constants.MAX_METERS_PER_SECOND * Constants.MAX_VOLTAGE,
        moduleStates[2].angle.getRadians());
    backRightModule.set(desiREDStates[0].speedMetersPerSecond / Constants.MAX_METERS_PER_SECOND * Constants.MAX_VOLTAGE,
        moduleStates[3].angle.getRadians());

  }

  public Pose2d getPose() {
    return poseVision.getEstimatedPosition();
  }

  public void resetOdometry() {
    // frontLeftModule.resetToAbsolute();
    // frontRightModule.resetToAbsolute();
    // backLeftModule.resetToAbsolute();
    // backRightModule.resetToAbsolute();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    moduleStates = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_METERS_PER_SECOND);
    
    // joystick_x.setDouble(RobotContainer.swerveStick.getRawAxis(Constants.X_AXIS_PORT));
    // joystick_y.setDouble(RobotContainer.swerveStick.getRawAxis(Constants.Y_AXIS_PORT));
    //if(DriverStation.isTeleop()) {
      frontLeftModule.set(moduleStates[0].speedMetersPerSecond / Constants.MAX_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[0].angle.getRadians());
      frontRightModule.set(moduleStates[1].speedMetersPerSecond / Constants.MAX_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[1].angle.getRadians());
      backLeftModule.set(moduleStates[2].speedMetersPerSecond / Constants.MAX_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[2].angle.getRadians());
      backRightModule.set(moduleStates[3].speedMetersPerSecond / Constants.MAX_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[3].angle.getRadians());//Negative introduced to new sds library  
    //}
   
    currentGyroAngle.setDouble(-gyroAngle().getDegrees());
    // currentPitch.setDouble(pitch().getDegrees());
    // currentRoll.setDouble(roll().getDegrees());

    m_odometry.update(gyroAngle(), new SwerveModulePosition[]{frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()});
    robotPosition = m_odometry.getPoseMeters();
    field.setRobotPose(poseVision.getEstimatedPosition());
    poseVision.update(gyroAngle(), new SwerveModulePosition[]{frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()});
    robot_x.setDouble(poseVision.getEstimatedPosition().getX());
    robot_y.setDouble(poseVision.getEstimatedPosition().getY());
  
  }
}