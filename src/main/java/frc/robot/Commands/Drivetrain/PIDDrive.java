// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDDriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Photonvision;

public class PIDDrive extends Command {
  private DriveSubsystem drivetrain;
  private double x;
  private double y;
  private double yaw;

  private static ProfiledPIDController yawPID = new ProfiledPIDController(
  DriveConstants.kYawP, DriveConstants.kYawI, DriveConstants.kYawD,
  new TrapezoidProfile.Constraints(
              DriveConstants.kMaxYawRateDegPerS,
              DriveConstants.kMaxYawAccelerationDegPerSSquared));

  private static ProfiledPIDController latPID = new ProfiledPIDController(
    DriveConstants.kYawP, DriveConstants.kYawI, DriveConstants.kYawD,
    new TrapezoidProfile.Constraints(
                DriveConstants.kMaxYawRateDegPerS,
                DriveConstants.kMaxYawAccelerationDegPerSSquared));

  private static ProfiledPIDController longPID = new ProfiledPIDController(
    DriveConstants.kYawP, DriveConstants.kYawI, DriveConstants.kYawD,
    new TrapezoidProfile.Constraints(
                DriveConstants.kMaxYawRateDegPerS,
                DriveConstants.kMaxYawAccelerationDegPerSSquared)); 

  /** Creates a new VisionDrive. */
  public PIDDrive(DriveSubsystem dt, double xErr, double yErr, double yawErr) {
    x = xErr;
    y = yErr;
    yaw = yawErr;

    drivetrain = dt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  public PIDDrive(DriveSubsystem dt) {
    drivetrain = dt;
  }

  public void setValues(double xErr, double yErr, double yawErr) {
    x = xErr;
    y = yErr;
    yaw = yawErr;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(latPID.calculate(x, PIDDriveConstants.latGoal),
    longPID.calculate(y, PIDDriveConstants.longGoal),
    yawPID.calculate(yaw, PIDDriveConstants.yawGoal), 
    false, 
    true);
  }

  public boolean atGoal() {
    if(latPID.atGoal() && longPID.atGoal() && yawPID.atGoal()) {
      return true;
    }
    else {
      return false;
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
