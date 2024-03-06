// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Climbers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class RunClimberManual extends Command {
  private Climber climber;
  private double velocity;
  /** Creates a new RunClimberManual. */
  public RunClimberManual(Climber climb, double speed) {
    climber = climb;
    velocity = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      climber.runAtVelocity(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
