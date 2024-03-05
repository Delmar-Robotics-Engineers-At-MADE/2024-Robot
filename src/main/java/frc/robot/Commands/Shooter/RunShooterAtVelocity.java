// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class RunShooterAtVelocity extends Command {
  private Shooter shooter;
  private double setpoint;
  private boolean end;
  private boolean compensate;
  /** Creates a new RunShooterAtVelocity. */
  public RunShooterAtVelocity(Shooter launchingDevice, double velocity, boolean compansation) {
    shooter = launchingDevice;
    setpoint = velocity;
    end = false;
    compensate = compansation;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launchingDevice);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!compensate) {
    shooter.runAtSpeed(setpoint);
    }
    else {
      shooter.runAtSpeed(setpoint*ShooterConstants.kCompenstion);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    end = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
