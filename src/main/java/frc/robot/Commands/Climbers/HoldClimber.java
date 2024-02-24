// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Climbers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class HoldClimber extends Command {
  private Climber climber;
  private double setpoint;
  /** Creates a new HoldClimber. */
  public HoldClimber(Climber climb) {
    climber = climb;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint = climber.getPos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.hold(setpoint);
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
