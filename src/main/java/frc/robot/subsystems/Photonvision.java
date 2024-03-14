// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photonvision extends SubsystemBase {
  private PhotonCamera backCam;
  private PhotonCamera fronCam;
  private Transform3d fake = new Transform3d();
  /** Creates a new Photonvision. */
  public Photonvision(NetworkTableInstance nt) {
    fronCam = new PhotonCamera(nt, "Microsoft_LifeCam_HD-3000");
    backCam = new PhotonCamera(nt, "HD_Pro_Webcam_C920");
    fronCam.setPipelineIndex(0);
  }
  // AprilTags
  public double[] getTagData(int id) {
    var result = backCam.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets) {
      PhotonTrackedTarget target = findCorrectTarget(id, result.getTargets());
      if (target.getFiducialId() == id) {
        double[] pack = {target.getBestCameraToTarget().getX(), 
        target.getBestCameraToTarget().getZ(),
        target.getBestCameraToTarget().getZ()};
        return pack;
      }
      else {
        double[] mt = {0};
        return mt;
    }
    }
    else {
      double[] mt = {0};
      return mt;
    }
  }

  public double[] getStageTagData(int[] ids) {
    var result = backCam.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets) {
      PhotonTrackedTarget target = findCorrectTarget(ids, result.getTargets());
      if (!(target.getFiducialId() == -1)) {
        double[] pack = {getTargetValues(target).getX(),
          getTargetValues(target).getZ(),
          getTargetValues(target).getRotation().getAngle()};
        return pack;
      }
      else {
        double[] mt = {0};
        return mt;
    }
    }
    else {
      double[] mt = {0};
      return mt;
    }
  }

  public double[] get3DTagData(int id) {
    var result = backCam.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets) {
      PhotonTrackedTarget target = findCorrectTarget(id, result.getTargets());
      if (!(target.getFiducialId() == -1)) {
        double[] pack = {getTargetValues(target).getX(),
          getTargetValues(target).getZ(),
          getTargetValues(target).getRotation().getAngle()};
        return pack;
      }
      else {
        double[] mt = {0};
        return mt;
    }
    }
    else {
      double[] mt = {0};
      return mt;
    }
  }




  public Transform3d getTargetValues(PhotonTrackedTarget target) {
    Transform3d targetVal = target.getBestCameraToTarget();
    return targetVal;
  }

  public boolean isObj() {
    var result = fronCam.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets) {
      return true;
    } else {
      return false;
    }
  }

  public double[] getObjData() {
    double data[] = {objYaw(), objPitch(), objSkew()};
    return data;
  }

  public double objYaw() {
    var result = fronCam.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    return target.getYaw();
  }

  public double objPitch() {
    var result = fronCam.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    return target.getPitch();
  }

  public double objSkew() {
    var result = fronCam.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    return target.getSkew();
  }

  public boolean isTarget(int targetID) {
    var result = backCam.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets) {
      PhotonTrackedTarget target = result.getBestTarget();
      if (targetID == 0) {
        return true;
      } else {
        if (targetID == target.getFiducialId()) {
          return true;
        } else {
          return false;
        }
      }
    } else {
      return false;
    }
  }

  private PhotonTrackedTarget findCorrectTarget(int id, List<PhotonTrackedTarget> lstTarget) {
    for(int x = 0; x<lstTarget.size(); x++) {
      if(lstTarget.get(x).getFiducialId() == id) {
        return lstTarget.get(x);
      }
    }
    return new PhotonTrackedTarget(0, 0, 0, 0, -1, fake, fake, 0.0, null, null);
  }

  private PhotonTrackedTarget findCorrectTarget(int[] ids, List<PhotonTrackedTarget> lstTarget) {
    for(int x = 0; x<lstTarget.size(); x++) {
      if(lstTarget.get(x).getFiducialId() == ids[0] || lstTarget.get(x).getFiducialId() == ids[1] || lstTarget.get(x).getFiducialId() == ids[2]) {
        return lstTarget.get(x);
      }
    }
    return new PhotonTrackedTarget(0, 0, 0, 0, -1, fake, fake, 0.0, null, null);
  }
}


