// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utils.Toolkit;
import frc.robot.subsystems.Shooter;

public class AccelerateShooter extends Command {
  private Shooter shooter;
  private double setpoint;
  private boolean end;
  /** Creates a new RunShooterAtVelocity. */
  public AccelerateShooter(Shooter launchingDevice, double velocity) {
    shooter = launchingDevice;
    end = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launchingDevice);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Toolkit.isInTolarance(shooter.getTopVelocity(), setpoint, ShooterConstants.kTolerance) && 
    Toolkit.isInTolarance(shooter.getBottomVelocity(), setpoint, ShooterConstants.kTolerance)) {
      end = true;
    }
    else {
      shooter.runAtSpeed(setpoint);
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
