// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Intake;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntakeOpenLoop extends Command {

  private Intake intake;
  private double speed;
  private boolean override;
  /** Creates a new RunIntakeOpenLoop. */
  public RunIntakeOpenLoop(Intake in, double input, boolean isOverride) {
    intake = in;
    speed = input;
    override = isOverride;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(in);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!override) {
      intake.hold();
    }
    else {
      intake.runOpenLoop(speed);
    }
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
