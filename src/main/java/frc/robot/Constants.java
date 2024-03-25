// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.1;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.0);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.0);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 12;
    public static final int kRearLeftTurningCanId = 13;
    public static final int kFrontRightTurningCanId = 11;
    public static final int kRearRightTurningCanId = 14;

    public static final boolean kGyroReversed = true;

    public static final double kYawP = 0.03;
    public static final double kYawI = 0.0;
    public static final double kYawD = 0.0;
    public static final double kMaxYawRateDegPerS = 8;
    public static final double kMaxYawAccelerationDegPerSSquared = 20;
    public static final double kYawToleranceDeg = 5;
    public static final double kYawRateToleranceDegPerS = 10;
    public static final double kLongToleranceMeter = 0.1;
    public static final double kLatToleranceMeter = 0.1;
  }

  public static final class PIDDriveConstants {
    public static final double latGoal = 0;
    public static final double longGoal = 0;
    public static final double yawGoal = 0;
  }
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ShooterConstants {
    public static final int kTopID = 8;
    public static final int kBottomID = 9;

    // init v should be 6.7 m/s for subwoofer at intake pos
    public static final double kSubwooferSpeed = 1340;

    // init v should be 8.2 m/s
    public static final double k3mSpeed = 1395;
    public static final double kAngleSpeed = 1500;

    public static final double kCompenstion = 1.6;

    public static final double kIdleSpeed = 700;
    public static final double kAmpSpeed = 900;
    public static final double kShuttleSpeed = 2500;
    public static final double kP = /*0.00025*/ 0;
    public static final double kI = 0.0;
    public static final double kD = /*0.000077*/ 0;
    public static final double kFF = 0.00017;
    public static final double kIz = 0;
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;
    public static final double kMaxRPM = 5676;
    public static final double kLaunchTime = 2;
    public static final double kThermalLimit = 48;
    public static final double kTolerance = 80;
  }

  public static final class IntakeConstants {
    public static final int kIntakeID = 10;
    public static final int kStarboardSensorDIOPort = 0;
    public static final int kPortSensorDIOPort = 1;
    public static final double kIntakeSpeed = 2500;
    public static final double kIntakeCaptureSpeed = 2100;

    public static final double kFeedSpeed = 3083;
    public static final double kReverseSpeed = 0.8;
    public static final double kCompenstion = 1.83;

    public static final double kP = 0.01;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.02;
    public static final double kIz = 0;
    public static final double kS = 0;
    public static final double kV = 2;
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;
    public static final double kMaxRPM = 11000;
    public static final double kVelTolerance = 400;

    public static final double kpP = 0.05;
    public static final double kpI = 0.0;
    public static final double kpD = 0.0;
    public static final double kpFF = 0;
  }

  public static final class ArmConstants {
    public static final int kLeftID = 6;
    public static final int kRightID = 7;
    public static final double kStowPos = 0.26;
    public static final double kDefaultPos = 0.032;
    // intake go lower
    public static final double kIntakePos = 0.0232;
    public static final double kSubwooferPos = 0.04;
    public static final double kAnglePos = 0.06;

    public static final double kFrontAmpPos = 0.18;

    // 3m position
    public static final double k3mPos = 0.064;

    public static final double kShuttlePos = 0.07;
    public static final double kBackAmpPos = 0.265;
    public static final double kUpperLimit = 0.35;
    public static final double kLowerLimit = 0.023;
    public static final double kOverrunLimit = 0;

    public static final double kP = 6; 
    public static final double kI = 0;
    public static final double kD = 0.7; 
    public static final double kIz = 0; 
    public static final double kFF = 0; 
    public static final double kMaxOutput = 0.7; 
    public static final double kMinOutput = -0.7;
    public static final double kMaxAccel = 0.18;
    public static final double kMaxVel = 0.85;

    public static final double kTolearance = 0.002;
    public static final double kManualSpeed = 0.3;

    public static final double kSVolts = 0;
    public static final double kGVolts = 0.33;
    public static final double kVVoltSecondPerRad = 6.24;
    public static final double kAVoltSecondSquaredPerRad = 0.04;
  }

  public static final class ClimberConstants {
    public static final int kPortID = 15;
    public static final int kStarboardID = 16;
    public static final int kPortDIO = 3;
    public static final int kStarboardDIO = 2;
    public static final double kP = 0.01; 
    public static final double kI = 0;
    public static final double kD = 0; 
    public static final double kIz = 0; 
    public static final double kFF = 0; 
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;

    public static final double kHomeSpeed = -0.4;
    // > 630, < 750
    public static final double kUpperLimit = 400;
    public static final double kLowerLimit = 5;
    public static final double kManualSpeed = 1500;
    public static final double kDirectSpeed = 0.5;
    public static final double kTolearance = 5;
    
  }

  // Cannot remember if the Extreme 3D PRO is 0 index or not.
  public static final class DriverConstants {
    public static final double kDefaultSpeed = 0.9;
    public static final double kYawSpeed = 0.7;
    public static final int kSetX = 4;
    public static final int kIntake = 1;
    public static final int kStowArm = 5;
    public static final int kAutoIntake = 3; 
    public static final int kAutoAmp = 6;
    public static final int kSelfDestruct = 9;
    public static final int kTurbo = 2;
  }

  public static final class OperatorConstants {
    public static final double kManoeuvreSpeed = 0.4; 
  }

  public static final class VisionConstants {
    public static final double kObjCamXOffset = 0.3302;
    public static final double kTagCamXOffset = 0.3302;
    public static final double kTagCamYOffset = 0.254;
  }

  public enum AprilTags {
    SUBSTATION_CLOSE,
    SUBSTATION_FAR,
    SPEAKER_CENTRE,
    SPEAKER_OFFSET,
    AMP,
    STAGE_LEFT,
    STAGE_RIGHT,
    STAGE_FAR
  }

  public enum States {
    STOW,
    INTAKE,
    CLOSE,
    FAR,
    AMP,
    CLIMB,
    DEFEND
  }

  public static final class FieldConstants {
    public static final double kSubwooferDepth = 0.917575;
    public static final double kChainDist = 0.422275;
  }

  public static final class LEDConstants {
    public static final double green = 0.77;
    public static final double purple = 0.91;
    public static final double red = -0.31;
    public static final double blue = -0.29;
    public static final double grey = -0.33;
  }
}
