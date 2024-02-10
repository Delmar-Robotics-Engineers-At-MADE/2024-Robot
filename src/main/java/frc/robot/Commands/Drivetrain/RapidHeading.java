// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;




public class RapidHeading extends Command {
  
  private static ProfiledPIDController m_PID = new ProfiledPIDController(
    DriveConstants.kYawP, DriveConstants.kYawI, DriveConstants.kYawD,
    new TrapezoidProfile.Constraints(
                DriveConstants.kMaxYawRateDegPerS,
                DriveConstants.kMaxYawAccelerationDegPerSSquared));

  private DriveSubsystem drivetrain;
  private double x;
  private double y;
  private int heading;
                
  /** Creates a new RapidHeading. */
  public RapidHeading(int target, double ySupplier, double xSupplier, DriveSubsystem dt) {
    drivetrain = dt;
    x = xSupplier;
    y = ySupplier;
    heading = target;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(x, y, m_PID.calculate(drivetrain.getHeading(), convertCardinalDirections(heading)), true, true);
  }

  
  public double convertCardinalDirections(int povAngleDeg) {
    // change d-pad values for left and right to specified angle
    if (povAngleDeg == 270) {
      povAngleDeg += 77;
    } else if (povAngleDeg == 90) {
      povAngleDeg -= 77;
    }
    // targetHeadingDegrees is counterclockwise so need to flip povAngle
    povAngleDeg = 360 - povAngleDeg;
    return povAngleDeg;
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
