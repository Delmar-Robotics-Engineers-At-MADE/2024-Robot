// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Commands.Drivetrain.RapidHeading;
import frc.robot.Commands.Intake.IntakeNoteAutomatic;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import javax.print.attribute.standard.MediaSize.NA;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Commands.Arm.HoldArm;
import frc.robot.Commands.Arm.RunArmClosedLoop;
import frc.robot.Commands.Arm.RunArmOpenLoop;

import frc.robot.Commands.Climbers.HomeClimber;
import frc.robot.Commands.Climbers.RunClimberManual;

import frc.robot.Commands.Drivetrain.RapidHeading;

import frc.robot.Commands.Intake.HoldIntake;
import frc.robot.Commands.Intake.IntakeNoteAutomatic;
import frc.robot.Commands.Intake.RunIntakeOpenLoop;

import frc.robot.Commands.Shooter.RunShooterAtVelocity;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Arm m_arm = new Arm(ArmConstants.kLeftID, ArmConstants.kRightID);
  private final Intake m_intake = new Intake(IntakeConstants.kIntakeID, IntakeConstants.kSensorDIOPort);
  private final Shooter m_shooter = new Shooter(ShooterConstants.kTopID, ShooterConstants.kBottomID);
  private final Climber m_portClimber = new Climber(ClimberConstants.kPortID, ClimberConstants.kPortDIO);
  private final Climber m_starboardClimber = new Climber(ClimberConstants.kStarboardID, ClimberConstants.kStarboardDIO);

  // The driver's controller
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
  CommandJoystick m_driverController = new CommandJoystick(OIConstants.kDriverControllerPort);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    m_driverController.setXChannel(1);
    m_driverController.setYChannel(0);
    m_driverController.setTwistChannel(2);

    Trigger override = m_operatorController.b();
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getTwist(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
    
    if (m_driverController.getHID().getPOV() != -1) {
        new RapidHeading(
            m_driverController.getHID().getPOV(),
            -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
            m_robotDrive);
    }

    override.whileTrue(
      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_operatorController.getLeftY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_operatorController.getLeftX(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_operatorController.getRightX(), OIConstants.kDriveDeadband),
          false, true), m_robotDrive));

    // Register Named Commands
    NamedCommands.registerCommand("intake", new IntakeNoteAutomatic(m_intake));
    NamedCommands.registerCommand("shootSubWoofer", new RunShooterAtVelocity(m_shooter, ShooterConstants.kSubwooferSpeed));
    NamedCommands.registerCommand("shootPodium", new RunShooterAtVelocity(m_shooter, ShooterConstants.kPodiumSpeed));
    NamedCommands.registerCommand("armToIntake", new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos));
    NamedCommands.registerCommand("armToSubwoofer", new RunArmClosedLoop(m_arm, ArmConstants.kSubwooferPos));
    NamedCommands.registerCommand("armToPodium", new RunArmClosedLoop(m_arm, ArmConstants.kPodiumPos));
    NamedCommands.registerCommand("armInside", new RunArmClosedLoop(m_arm, ArmConstants.kStowPos));
    NamedCommands.registerCommand("homePort", new HomeClimber(m_portClimber));
    NamedCommands.registerCommand("homeStarboard", new HomeClimber(m_starboardClimber));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController.getHID(), 5)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    }
    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}
