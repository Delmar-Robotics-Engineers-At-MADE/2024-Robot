// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Photonvision;

public class AutoIntake extends PIDDrive {
  
  private Intake intake;
  private boolean end;
  /** Creates a new AutoIntake. */
  public AutoIntake(DriveSubsystem dt, Intake in, Photonvision pv) {
    super(dt, pv.getObjData()[0], pv.getObjData()[1] , pv.getObjData()[2]);
    intake = in;
    end = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!intake.isNote()) {
      intake.autoIntake();
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
    return end;
  }
}
