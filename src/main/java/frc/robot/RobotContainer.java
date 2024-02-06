// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Commands.Drivetrain.Drive;
import frc.robot.Commands.Drivetrain.RapidHeading;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Commands.Warning;
import frc.robot.Commands.Arm.HoldArm;
import frc.robot.Commands.Arm.RunArmClosedLoop;
import frc.robot.Commands.Climbers.HoldClimber;
import frc.robot.Commands.Climbers.HomeClimber;
import frc.robot.Commands.Climbers.RunClimberManual;

import frc.robot.Commands.Intake.Feed;
import frc.robot.Commands.Intake.HoldIntake;
import frc.robot.Commands.Intake.IntakeNoteAutomatic;
import frc.robot.Commands.Intake.RunIntakeOpenLoop;
import frc.robot.Commands.Shooter.AccelerateShooter;
import frc.robot.Commands.Shooter.RunShooterAtVelocity;
import frc.robot.Commands.Shooter.ShootNote;

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

  private final Dashboard dashboard;

  // Command Groups
  ParallelCommandGroup feedAndShootSubwoofer = new ParallelCommandGroup(
    new ShootNote(m_shooter, ShooterConstants.kSubwooferSpeed),
    new Feed(m_intake)
  );
  ParallelCommandGroup feedAndShootPodium = new ParallelCommandGroup(
    new ShootNote(m_shooter, ShooterConstants.k3mSpeed),
    new Feed(m_intake)
  );
  SequentialCommandGroup shootSubwoofer = new SequentialCommandGroup(
    new RunArmClosedLoop(m_arm, ArmConstants.kSubwooferPos),
    new AccelerateShooter(m_shooter, ShooterConstants.kSubwooferSpeed),
    feedAndShootSubwoofer
  );
  SequentialCommandGroup shootPodium = new SequentialCommandGroup(
    new RunArmClosedLoop(m_arm, ArmConstants.k3mPos),
    new AccelerateShooter(m_shooter, ShooterConstants.k3mSpeed),
    feedAndShootPodium
  );
  ParallelCommandGroup intake = new ParallelCommandGroup(
    new IntakeNoteAutomatic(m_intake),
    new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos)
  );

  ParallelCommandGroup homeClimbers = new ParallelCommandGroup(
    new HomeClimber(m_portClimber),
    new HomeClimber(m_starboardClimber)
  );

  ParallelCommandGroup forceFeed = new ParallelCommandGroup(
    new RunIntakeOpenLoop(m_intake, -IntakeConstants.kReverseSpeed),
    new RunShooterAtVelocity(m_shooter, ShooterConstants.kAmpSpeed)
  );
  ParallelCommandGroup forceReverse = new ParallelCommandGroup(
    new RunIntakeOpenLoop(m_intake, IntakeConstants.kReverseSpeed),
    new RunShooterAtVelocity(m_shooter, -ShooterConstants.kAmpSpeed)
  );

  private final SendableChooser<Command> autoChooser;

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    m_driverController.setXChannel(1);
    m_driverController.setYChannel(0);
    m_driverController.setTwistChannel(2);



    configureButtonBindings();

    // Register Named Commands
    NamedCommands.registerCommand("intake", intake);
    NamedCommands.registerCommand("shootSubWoofer", shootSubwoofer);
    NamedCommands.registerCommand("shootPodium", shootPodium);
    NamedCommands.registerCommand("armInside", new RunArmClosedLoop(m_arm, ArmConstants.kStowPos));
    NamedCommands.registerCommand("homePort", new HomeClimber(m_portClimber));
    NamedCommands.registerCommand("homeStarboard", new HomeClimber(m_starboardClimber));
    NamedCommands.registerCommand("armDown", new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    dashboard = new Dashboard(m_robotDrive, m_arm, m_intake, m_shooter, m_portClimber, m_starboardClimber, autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(

        new Drive(
          m_driverController.getX(), 
          m_driverController.getY(), 
          m_driverController.getTwist(), 
        true, true, m_robotDrive, DriverConstants.kDefaultSpeed));

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new Drive(m_operatorController.getLeftX(), 
          m_operatorController.getLeftY(), 
          m_operatorController.getRightX(), 
          true, true, m_robotDrive, OperatorConstants.kManoeuvreSpeed));
    m_driverController.povCenter().whileFalse(
        new RapidHeading(
            m_driverController.getHID().getPOV(),
            -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
            m_robotDrive));

    m_driverController.button(2).whileTrue(
      new Drive(m_operatorController.getLeftX(),
      m_operatorController.getLeftY(),
      m_operatorController.getRightX(),
      false, true, m_robotDrive));
    

    m_operatorController.start().whileTrue(
      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_operatorController.getLeftY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_operatorController.getLeftX(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_operatorController.getRightX(), OIConstants.kDriveDeadband),
          false, true), m_robotDrive));

    // Subsystem Default Commands
    m_intake.setDefaultCommand(new HoldIntake(m_intake));
    m_arm.setDefaultCommand(new HoldArm(m_arm));
    m_shooter.setDefaultCommand(new RunShooterAtVelocity(m_shooter, ShooterConstants.kIdleSpeed));
    m_portClimber.setDefaultCommand(new HoldClimber(m_portClimber));
    m_starboardClimber.setDefaultCommand(new HoldClimber(m_starboardClimber));


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
    m_driverController.button(DriverConstants.kSetX)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    m_driverController.button(DriverConstants.kIntake).whileTrue(intake);
    m_driverController.button(DriverConstants.kHoldArmDown).toggleOnTrue(new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos));

    m_operatorController.start().whileTrue(new Warning("¡OVERRIDE!"));
    m_operatorController.start().and(m_operatorController.povUp()).whileTrue(new RunArmClosedLoop(m_arm, ArmConstants.kManualSpeed));
    m_operatorController.start().and(m_operatorController.povDown()).whileTrue(new RunArmClosedLoop(m_arm, -ArmConstants.kManualSpeed));
    m_operatorController.start().and(m_operatorController.povRight()).whileTrue(forceFeed);
    m_operatorController.start().and(m_operatorController.povLeft()).whileTrue(forceReverse);

    m_operatorController.back().whileTrue(homeClimbers);
    m_operatorController.rightBumper().whileTrue(new RunClimberManual(m_starboardClimber, ClimberConstants.kManualSpeed));
    m_operatorController.rightTrigger().whileTrue(new RunClimberManual(m_portClimber, -ClimberConstants.kManualSpeed));
    m_operatorController.leftBumper().whileTrue(new RunClimberManual(m_portClimber, ClimberConstants.kManualSpeed));
    m_operatorController.leftTrigger().whileTrue(new RunClimberManual(m_portClimber, ClimberConstants.kManualSpeed));

    }
    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
