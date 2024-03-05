// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;

public class Feed extends Command {
  private Intake intake;
  private double startTime;
  private double timeout;
  /** Creates a new Feed. */
  public Feed(Intake in) {
    intake = in;
    startTime = 0.0;
    timeout = ShooterConstants.kLaunchTime;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(in);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //intake.runAtVelocity(IntakeConstants.kFeedSpeed);
    intake.runAtVelocity(IntakeConstants.kFeedSpeed);
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
