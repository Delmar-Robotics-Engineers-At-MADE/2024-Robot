// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Photonvision;


public class AutoFire3D extends PIDDrive {

  private Photonvision photon;
  private int tag;
  private boolean end;
  private SequentialCommandGroup cmd;
  /** Creates a new AutoFire. */
  public AutoFire3D(DriveSubsystem dt, Photonvision pv, int tagID, 
    SequentialCommandGroup sequence) {
    super(dt, VisionConstants.kTagCamXOffset, VisionConstants.kTagCamYOffset);
    cmd = sequence;
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
      if (x != 0) {x = (1 - x)*15;}  // MJS: zero means no target, so leave it zero
      double y = 0;//photon.get3DTagData(tag)[0]*10;
      double z = photon.get3DTagData(tag)[1]*30;
      this.setValues(x, y, z); // MJS: minus Z, and multiply x/y, and swap them
      super.execute();
      if(this.atGoal()) {
      cmd.execute();
      end = cmd.isFinished();
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
