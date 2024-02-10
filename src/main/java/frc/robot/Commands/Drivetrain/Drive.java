// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Drive extends Command {
  private DriveSubsystem drivetrain;
  private double x;
  private double y;
  private double z;
  private boolean fieldRel;
  private boolean srl;
  private double multiplier;
  /** Creates a new Drive. */
  public Drive(double xSource, double ySource, double zSource, boolean fieldRelative, boolean rateLimit, DriveSubsystem dt, double speedModifier) {
    drivetrain = dt;
    x = xSource;
    y = ySource;
    z = zSource;
    fieldRel = fieldRelative;
    srl = rateLimit;
    multiplier = speedModifier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

    public Drive(double xSource, double ySource, double zSource, boolean fieldRelative, boolean rateLimit, DriveSubsystem dt) {
    drivetrain = dt;
    x = xSource;
    y = ySource;
    z = zSource;
    fieldRel = fieldRelative;
    srl = rateLimit;
    multiplier = 1.0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new RunCommand( 
      () ->
    drivetrain.drive(-MathUtil.applyDeadband(x, OIConstants.kDriveDeadband) * multiplier, 
    -MathUtil.applyDeadband(y, OIConstants.kDriveDeadband) * multiplier, 
    -MathUtil.applyDeadband(z, OIConstants.kDriveDeadband) * multiplier, fieldRel, srl));
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
