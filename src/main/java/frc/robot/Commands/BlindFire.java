// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Arm.RunArmClosedLoop;
import frc.robot.Commands.Intake.Feed;
import frc.robot.Commands.Shooter.RunShooterAtVelocity;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class BlindFire extends Command {
  /** Creates a new BlindFire. */
  private final Arm arm;
  private final Shooter shooter;
  private final Intake intake;
  private final double shooterSetpoint;
  private final double armSetpoint;
  private boolean end;
  public BlindFire(Arm arm, Shooter shooter, Intake intake, double position, double speed) {
    this.arm = arm;
    this.shooter = shooter;
    this.intake = intake;
    armSetpoint = position;
    shooterSetpoint = speed;
    end = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
          RunArmClosedLoop aCMD = new RunArmClosedLoop(arm, armSetpoint);
      if(aCMD.isFinished()) {
        RunShooterAtVelocity wheeCMD = new RunShooterAtVelocity(shooter, shooterSetpoint, true);
        Feed feed = new Feed(intake);
        if(wheeCMD.isFinished() && feed.isFinished()) {
          end = true;
        }
        else {
          end = false;
        }
      }
      else {
        end = false;
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
