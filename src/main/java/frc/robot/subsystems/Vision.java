// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.Pair;
//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.cscore.AxisCamera;
//import edu.wpi.first.apriltag.AprilTagFields;
//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.cscore.MjpegServer;
//import edu.wpi.first.cscore.UsbCamera;
//import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Quaternion;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
//import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
//import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
//import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
//import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  PhotonCamera camera = new PhotonCamera("USB_CAMERA");
  PhotonPipelineResult result;

  AprilTagFieldLayout aprilTagFieldLayout;
  protected PhotonPoseEstimator photonPoseEstimator;
  Optional<EstimatedRobotPose> robotPos;


  //AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact);
  Transform3d cameraToRobot = new Transform3d(new Pose3d(0, 0, 0, new Rotation3d()), new Pose3d(.29718, .0762, .51054, new Rotation3d()));
  GenericEntry xPos = Shuffleboard.getTab("Position").add("X-Value", 0).getEntry();
  GenericEntry yPos = Shuffleboard.getTab("Position").add("Y-Value", 0).getEntry();
  GenericEntry testXPos = Shuffleboard.getTab("Position").add("Test-X-Value", 0).getEntry();
  GenericEntry testYPos = Shuffleboard.getTab("Position").add("Test-Y-Value", 0).getEntry();
  GenericEntry yawPos = Shuffleboard.getTab("Position").add("Yaw", 0).getEntry();
  Field2d field = new Field2d();
  ComplexWidget fieldShuffleboard = Shuffleboard.getTab("Position").add("2023-Field", field).withWidget(BuiltInWidgets.kField).withProperties(Map.of("robot icon size", 20)).withSize(5, 3);


  //ComplexWidget CameraShuffleboard = Shuffleboard.getTab("2022Robot").add("Camera", cam).withWidget(BuiltInWidgets.kCameraStream);
  //UsbCamera cam = CameraServer.startAutomaticCapture(0);
  //CameraServer cam = new MjpegServer("stream.mjpg", "http://10.26.43.11/", 1182);
  //MjpegServer cam = new MjpegServer("stream.mjpg", "http://10.26.43.11/", 1182);
  //MjpegServer cam = new MjpegServer("USB_CAMERA", "http://10.26.43.11/", 1182);
  //MjpegServer cam = new MjpegServer("USB_CAMERA", 1182);
  //AxisCamera cam = CameraServer.addAxisCamera("photonvision.local:1182");

  //ComplexWidget CameraShuffleboard = Shuffleboard.getTab("2022Robot").add("Camera", cam).withWidget(BuiltInWidgets.kCameraStream);


  //PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, cameraToRobot);
  //Pose2d visionPos;
  PhotonTrackedTarget idTwoTarget;
  

  
  public Vision() {
    //photonPoseEstimator.setReferencePose(new Pose2d());
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, cameraToRobot);
    } catch (Exception e) {
      System.out.println("Error with AprilTagFieldLayout");
    }
  }

  @Override
  public void periodic() {
    result = camera.getLatestResult();
    
    if(result.hasTargets()) {
      //photonPoseEstimator.setReferencePose(new Pose2d());
      idTwoTarget = result.getBestTarget();      
      xPos.setDouble(idTwoTarget.getBestCameraToTarget().getX());
      yPos.setDouble(idTwoTarget.getBestCameraToTarget().getY());
      robotPos = photonPoseEstimator.update();
      

      testXPos.setDouble(robotPos.get().estimatedPose.toPose2d().getX());
      testYPos.setDouble(robotPos.get().estimatedPose.toPose2d().getY());
      // yawPos.setDouble(result.getBestTarget().getBestCameraToTarget().getRotation().toRotation2d().getDegrees());
      field.setRobotPose(robotPos.get().estimatedPose.toPose2d());
    }

    
  }
}