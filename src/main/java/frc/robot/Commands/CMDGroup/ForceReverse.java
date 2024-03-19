// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CMDGroup;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Commands.Intake.RunIntakeOpenLoop;
import frc.robot.Commands.Shooter.RunShooterAtVelocity;

public class ForceReverse extends Command {
  private final RunIntakeOpenLoop incmd;
  private final RunShooterAtVelocity shcmd;
  /** Creates a new ForceFeed. */
  public ForceReverse(Intake intake, Shooter shooter) {
    incmd = new RunIntakeOpenLoop(intake, -IntakeConstants.kReverseSpeed);
    shcmd = new RunShooterAtVelocity(shooter, ShooterConstants.k3mSpeed, false);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    incmd.execute();
    shcmd.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return incmd.isFinished() && shcmd.isFinished();
  }
}
