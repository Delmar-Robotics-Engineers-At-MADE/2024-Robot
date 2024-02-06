// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;


import frc.robot.Commands.Arm.RunArmClosedLoop;
import frc.robot.Commands.Intake.Feed;
import frc.robot.Commands.Shooter.RunShooterAtVelocity;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.Shooter;


public class AutoFire3D extends PIDDrive {

  private Arm arm;
  private Intake intake;
  private Shooter whee;
  private Photonvision photon;

  private double armSetpoint;
  private double wheeSeptoint;
  private int tag;
  private boolean end;
  /** Creates a new AutoFire. */
  public AutoFire3D(DriveSubsystem dt, Photonvision pv, int tagID, 
    Arm ar, Intake in, Shooter sh, double armPos, double shooterVel) {
    super(dt);
    arm = ar;
    intake = in;
    whee = sh;
    photon = pv;

    armSetpoint = armPos;
    wheeSeptoint = shooterVel;
    tag = tagID;
    end = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt, pv, ar, in, sh);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(photon.isTarget(tag)) {
      this.setValues(photon.getTargetValues().getX(), photon.getTargetValues().getY(), photon.getTargetValues().getRotation().getAngle());
      if(this.atGoal()) {
      RunArmClosedLoop aCMD = new RunArmClosedLoop(arm, armSetpoint);
      if(aCMD.isFinished()) {
        RunShooterAtVelocity wheeCMD = new RunShooterAtVelocity(whee, wheeSeptoint);
        Feed feed = new Feed(intake);
        if(wheeCMD.isFinished() && feed.isFinished()) {
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
      else {
        end = false;
      }
    }
    else {
      end = true;
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
