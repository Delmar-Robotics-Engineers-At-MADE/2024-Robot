// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CMDGroup;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Intake.Feed;
import frc.robot.Commands.Shooter.ShootNote;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class FeedAndShootPodium extends Command {
  private final Shooter shooter;
  private final Intake intake;
  /** Creates a new FeedAndShootPodium. */
  public FeedAndShootPodium(Shooter shooter, Intake intake) {
    this.shooter = shooter;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new ShootNote(shooter, ShooterConstants.k3mSpeed);
    new Feed(intake);
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
