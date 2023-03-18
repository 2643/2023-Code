// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
//import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
  double[] targetInfo = new double[6];
  //Field2d field = new Field2d();
  public Pose2d visionPos = new Pose2d();
  NetworkTableEntry hasTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
  NetworkTableEntry targetInfoEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue");
  NetworkTableEntry cl = NetworkTableInstance.getDefault().getTable("limelight").getEntry("cl");
  NetworkTableEntry tl = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl");
  public double visionLatency;
  //ComplexWidget fieldShuffleboard = Shuffleboard.getTab("Field-New").add("2023-Field", field).withWidget(BuiltInWidgets.kField).withProperties(Map.of("robot icon size", 20));
  GenericEntry targetXEntry = Shuffleboard.getTab("Debug").add("Vision-X", 0).getEntry();
  GenericEntry targetYEntry = Shuffleboard.getTab("Debug").add("Vision-Y", 0).getEntry();
  GenericEntry usingVision = Shuffleboard.getTab("Debug").add("Using Vision", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();



  /** Creates a new Vision
   * . */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (hasTarget.getDouble(0) == 1) {
      targetInfo = targetInfoEntry.getDoubleArray(new double[6]);
      visionPos = new Pose2d(targetInfo[0], targetInfo[1], new Rotation2d(Math.toRadians(targetInfo[5] - 90)));
      //field.setRobotPose(targetInfo[0], targetInfo[1], new Rotation2d(Math.toRadians(targetInfo[5] - 90)));
      visionLatency = cl.getDouble(0) + tl.getDouble(0);
      RobotContainer.m_drivetrain.poseVision.addVisionMeasurement(visionPos, Timer.getFPGATimestamp() - (tl.getDouble(0)/1000.0) - (cl.getDouble(0)/1000.0));
      targetXEntry.setDouble(targetInfo[0]);
      targetYEntry.setDouble(targetInfo[1]);
      usingVision.setBoolean(true);
    } else {
      usingVision.setBoolean(false);
      targetXEntry.setDouble(-1);
      targetYEntry.setDouble(-1);
    }
    if(RobotContainer.coneMode.getAsBoolean()) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    } else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }
  }
}
