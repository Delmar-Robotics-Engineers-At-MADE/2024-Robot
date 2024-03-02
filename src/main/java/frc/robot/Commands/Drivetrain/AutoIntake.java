// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import frc.robot.Commands.Arm.RunArmClosedLoop;
import frc.robot.Commands.Intake.HoldIntake;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Photonvision;

public class AutoIntake extends PIDDrive {
  
  private Arm arm;
  private DriveSubsystem drivetrain;
  private Intake intake;
  private Photonvision pCam;
  private boolean end;
  /** Creates a new AutoIntake. */
  public AutoIntake(DriveSubsystem dt, Intake in, Photonvision pv, Arm ar) {
    super(dt, VisionConstants.kObjCamXOffset, 0.0);
    intake = in;
    pCam = pv;
    arm = ar;
    end = false;
    drivetrain = dt;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt, in, pv);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(pCam.isObj()) {
      new RunArmClosedLoop(arm, ArmConstants.kIntakePos);
      this.setValues(0, 0, pCam.objYaw());
      super.execute();
      if(this.atGoal()) {
        drivetrain.drive(OperatorConstants.kManoeuvreSpeed, 0, 0, false, true); 
        if(!intake.isNote()) {
          intake.autoIntake();
        }
        else {
          end = true;
        }
      }
    }
    else {
      end = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    new RunArmClosedLoop(arm, ArmConstants.kIntakePos);
    new HoldIntake(intake);
    return end;
  }
}
