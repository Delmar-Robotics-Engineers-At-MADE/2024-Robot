// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Climbers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Utils.Toolkit;
import frc.robot.subsystems.Climber;

public class RunClimberToPos extends Command {
  private Climber climber;
  private double setpoint;
  private boolean end;
  /** Creates a new RunClimberToPos. */
  public RunClimberToPos(Climber climb, double target) {
    climber = climb;
    setpoint = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!Toolkit.isInTolarance(climber.getPos(), setpoint, ClimberConstants.kTolearance)) {
      climber.runToPos(setpoint);
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
