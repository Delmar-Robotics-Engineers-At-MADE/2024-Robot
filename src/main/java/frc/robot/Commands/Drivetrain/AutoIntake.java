// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import frc.robot.Commands.Arm.RunArmClosedLoop;
import frc.robot.Commands.Intake.HoldIntake;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Photonvision;

public class AutoIntake extends PIDDrive {
  
  private Arm arm;
  private Intake intake;
  private Photonvision pCam;
  private boolean end;
  /** Creates a new AutoIntake. */
  public AutoIntake(DriveSubsystem dt, Intake in, Photonvision pv, Arm ar) {
    super(dt);
    intake = in;
    pCam = pv;
    arm = ar;
    end = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt, in, pv);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(pCam.checkForObj()) {
      new RunArmClosedLoop(arm, ArmConstants.kIntakePos);
      this.setValues(pCam.getObjData()[0], pCam.getObjData()[1], pCam.getObjData()[2]);
      if(!intake.isNote()) {
        intake.autoIntake();
      }
      else {
        end = true;
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
