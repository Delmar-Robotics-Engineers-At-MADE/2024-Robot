// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.Climbers.RunClimberToPos;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utils.Toolkit;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Photonvision;

public class AutoClimb extends PIDDrive {
  /** Creates a new AutoClimb. */
  private final DriveSubsystem drivetrain;
  private final Climber port;
  private final Climber starboard;
  private final Photonvision pCam;
  private boolean end;
  public AutoClimb(DriveSubsystem dt, Climber port, Climber starboard, Photonvision photonvision) {
    super(dt, photonvision.getStageTagData(Toolkit.getStageFiducialIDs())[0], photonvision.getStageTagData(Toolkit.getStageFiducialIDs())[1], photonvision.getStageTagData(Toolkit.getStageFiducialIDs())[2], VisionConstants.kTagCamXOffset, VisionConstants.kTagCamYOffset);
    drivetrain = dt;
    this.port = port;
    this.starboard = starboard;
    pCam = photonvision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt, port, starboard);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new ParallelCommandGroup(
      new RunClimberToPos(port, ClimberConstants.kUpperLimit),
      new RunClimberToPos(starboard, ClimberConstants.kUpperLimit)
    );
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
