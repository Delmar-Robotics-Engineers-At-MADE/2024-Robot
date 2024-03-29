// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;


import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Photonvision;


public class AutoFire3D extends PIDDrive {

  private Photonvision photon;
  private int tag;
  private boolean end;
  /** Creates a new AutoFire. */
  public AutoFire3D(DriveSubsystem dt, Photonvision pv, int tagID) {
    super(dt, VisionConstants.kTagCamXOffset, VisionConstants.kTagCamYOffset);
    photon = pv;
    tag = tagID;
    end = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt, pv);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!(photon.get3DTagData(tag).length == 0)) {
      double x = photon.get3DTagData(tag)[0];
      if (x != 0) {x = (VisionConstants.kTagCamXOffset + 1.3 - x)*15;}  // MJS: zero means no target, so leave it zero
      double y = 0;//photon.get3DTagData(tag)[0]*10;
      double z = photon.get3DTagData(tag)[1]*30;
      if (z != 0) {z = (VisionConstants.kTagCamYOffset + 1.3 - x)*15;}
      this.setValues(x, y, z); // MJS: minus Z, and multiply x/y, and swap them
      super.execute();
      if(this.atGoal()) {
        end = true;
      }
      else {
        end = false;
      }
      }
      else {
        end = false;
      }
    }
  

  // Callesd once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
