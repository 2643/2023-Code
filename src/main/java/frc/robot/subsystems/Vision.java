// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  PhotonCamera camera = new PhotonCamera("photonvision2");

  PhotonPipelineResult result = camera.getLatestResult();

  
  public Vision() {}

  @Override
  public void periodic() {
    
    //target = result.getBestTarget();

    System.out.println(result.getBestTarget());

    // PhotonTrackedTarget target = result.getBestTarget();
    // System.out.println(target.getArea());
  }
}
