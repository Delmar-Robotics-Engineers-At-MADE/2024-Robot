// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDDriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class PIDDrive extends Command {
  private final DriveSubsystem drivetrain;
  private double x;
  private double y;
  private double yaw;
  private final double cameraOffset;
  private final double cameraDepth;

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
  public PIDDrive(DriveSubsystem dt, double xErr, double yErr, double yawErr, double cameraOffset, double cameraDepth) {
    x = xErr;
    y = yErr;
    yaw = yawErr;
    this.cameraOffset = cameraOffset;
    this.cameraDepth = cameraDepth;
    yawPID.setTolerance(DriveConstants.kYawToleranceDeg);
    latPID.setTolerance(DriveConstants.kLatToleranceMeter);
    longPID.setTolerance(DriveConstants.kLongToleranceMeter);

    drivetrain = dt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  public PIDDrive(DriveSubsystem dt, double cameraOffset, double cameraDepth) {
    drivetrain = dt;
    this.cameraOffset = cameraOffset;
    this.cameraDepth = cameraDepth;
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
    System.out.println("MJS: executing PIDDrive: " + latPID.calculate(x, PIDDriveConstants.latGoal) + " " + longPID.calculate(y, PIDDriveConstants.longGoal) + " " + yawPID.calculate(yaw, PIDDriveConstants.yawGoal));
    drivetrain.drive(latPID.calculate(x + cameraOffset, PIDDriveConstants.latGoal),
    longPID.calculate(y + cameraDepth, PIDDriveConstants.longGoal),
    yawPID.calculate(yaw, PIDDriveConstants.yawGoal), 
    false, 
    true);
  }

  public boolean atGoal() {
    if(latPID.atGoal() && longPID.atGoal() && yawPID.atGoal()) {
      return true;
    }
    else {
      System.out.println("MJS: PIDDrive: not at goal: " + latPID.atGoal() + " " + longPID.atGoal() + " " + yawPID.atGoal() + " " + yawPID.getPositionError() + " " + yawPID.getPositionTolerance());
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




  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //   System.out.println("MJS: executing PIDDrive: " + latPID.calculate(x, PIDDriveConstants.latGoal) + " " + longPID.calculate(y, PIDDriveConstants.longGoal) + " " + yawPID.calculate(yaw, PIDDriveConstants.yawGoal));
  //   drivetrain.drive(latPID.calculate(x, PIDDriveConstants.latGoal),
  //   longPID.calculate(y, PIDDriveConstants.longGoal),
  //   yawPID.calculate(yaw, PIDDriveConstants.yawGoal), 
  //   false, 
  //   true);
  // }

  // public boolean atGoal() {
  //   if(latPID.atGoal() && longPID.atGoal() && yawPID.atGoal()) {
  //     return true;
  //   }
  //   else {
  //     System.out.println("MJS: PIDDrive: not at goal: " + latPID.atGoal() + " " + longPID.atGoal() + " " + yawPID.atGoal() + " " + yawPID.getPositionError() + " " + yawPID.getPositionTolerance());
  //     return false;
  //   }
  // }