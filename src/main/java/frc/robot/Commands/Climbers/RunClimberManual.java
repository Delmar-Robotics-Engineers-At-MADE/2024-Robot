// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Climbers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class RunClimberManual extends Command {
  private Climber climber;
  private double velocity;
  private boolean override;
  /** Creates a new RunClimberManual. */
  public RunClimberManual(Climber climb, double speed, boolean isOverride) {
    climber = climb;
    velocity = speed;
    override = isOverride;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!override) {
      if(!climber.isHomed()) {
        isFinished();
      }
      else {
        climber.runOpenLoop(velocity);
      }
    }
    else {
      climber.runOpenLoop(velocity);
      System.out.println("¡OVERRIDE! ¡EXCERCISE CAUTION!");
    }
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
