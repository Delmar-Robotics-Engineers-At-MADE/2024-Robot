// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class ShootNote extends Command {
  private Shooter shooter;
  private double setpoint;
  private double timeout;
  private double startTime;
  /** Creates a new RunShooterAtVelocity. */
  public ShootNote(Shooter launchingDevice, double velocity) {
    shooter = launchingDevice;
    setpoint = velocity;
    startTime = 0.0;
    timeout = ShooterConstants.kLaunchTime;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launchingDevice);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooter.runAtSpeed(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double elapsed = Timer.getFPGATimestamp() - startTime;
    if(elapsed >= timeout) {
      return false;
    }
    else {
      return true;
    }
  }
}
