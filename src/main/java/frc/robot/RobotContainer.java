// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utils.Dashboard;
import frc.robot.Utils.Toolkit;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Commands.Arm.HoldArm;
import frc.robot.Commands.Arm.RunArmClosedLoop;
import frc.robot.Commands.Arm.RunArmOpenLoop;
import frc.robot.Commands.CMDGroup.ForceFeed;
import frc.robot.Commands.CMDGroup.ForceReverse;
import frc.robot.Commands.Climbers.HoldClimber;
import frc.robot.Commands.Climbers.HomeClimber;
import frc.robot.Commands.Climbers.RunClimberNormalLaw;

import frc.robot.Commands.Intake.Feed;
import frc.robot.Commands.Intake.IntakeNoteAutomatic;
import frc.robot.Commands.Intake.RunIntakeOpenLoop;
import frc.robot.Commands.Shooter.AccelerateShooter;
import frc.robot.Commands.Shooter.RunShooterAtVelocity;
import frc.robot.Commands.Shooter.RunShooterEternal;

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
  private final Intake m_intake = new Intake(IntakeConstants.kIntakeID, IntakeConstants.kPortSensorDIOPort, IntakeConstants.kStarboardSensorDIOPort);
  private final Shooter m_shooter = new Shooter(ShooterConstants.kTopID, ShooterConstants.kBottomID);
  private final Climber m_portClimber = new Climber(ClimberConstants.kPortID, ClimberConstants.kPortDIO, true);
  private final Climber m_starboardClimber = new Climber(ClimberConstants.kStarboardID, ClimberConstants.kStarboardDIO, false);
  private final Blinkin blinkin = Blinkin.getInstance();

  // The driver's controller
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
  CommandJoystick m_driverController = new CommandJoystick(OIConstants.kDriverControllerPort);
  //DriverCommandXboxController m_driverControllerX = new DriverCommandXboxController(2);

  private final Dashboard dashboard;


  private boolean override = false;

  // Command Groups
  ParallelCommandGroup intake = new ParallelCommandGroup(
    new IntakeNoteAutomatic(m_intake),
    new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos)
  );

  ParallelCommandGroup homeClimbers = new ParallelCommandGroup(
    new HomeClimber(m_portClimber),
    new HomeClimber(m_starboardClimber)
  );

  ForceFeed forceFeed = new ForceFeed(m_intake, m_shooter);

  ParallelCommandGroup forceReverse = new ParallelCommandGroup(
    new RunIntakeOpenLoop(m_intake, IntakeConstants.kReverseSpeed),
    new RunShooterAtVelocity(m_shooter, ShooterConstants.kAmpSpeed, true)
  );


  // Firing Sequences
  SequentialCommandGroup subwooferFire = new SequentialCommandGroup(
        Toolkit.sout("sFire init"),
          new RunArmClosedLoop(m_arm, ArmConstants.kSubwooferPos),
          new AccelerateShooter(m_shooter, ShooterConstants.kSubwooferSpeed),
        Toolkit.sout("shoot init"),
        new ParallelRaceGroup(
          new WaitCommand(ShooterConstants.kLaunchTime),
          new RunShooterEternal(m_shooter, ShooterConstants.kSubwooferSpeed, true),
          new Feed(m_intake)
        ),
        Toolkit.sout("sFire end")
      );

  SequentialCommandGroup distanceFire = new SequentialCommandGroup(
        Toolkit.sout("Fire init"),
          new RunArmClosedLoop(m_arm, ArmConstants.k3mPos),
          new AccelerateShooter(m_shooter, ShooterConstants.k3mSpeed),
        Toolkit.sout("shoot init"),
        new ParallelRaceGroup(
          new WaitCommand(ShooterConstants.kLaunchTime),
          new RunShooterEternal(m_shooter, ShooterConstants.k3mSpeed, true),
          new Feed(m_intake)
        ),
        Toolkit.sout("Fire end")
      );

  SequentialCommandGroup shuttleFire = new SequentialCommandGroup(
        Toolkit.sout("Fire init"),
          new RunArmClosedLoop(m_arm, ArmConstants.kShuttlePos),
          new AccelerateShooter(m_shooter, ShooterConstants.kShuttleSpeed),
        Toolkit.sout("shoot init"),
        new ParallelRaceGroup(
          new WaitCommand(ShooterConstants.kLaunchTime),
          new RunShooterEternal(m_shooter, ShooterConstants.kShuttleSpeed, true),
          new Feed(m_intake)
        ),
        Toolkit.sout("Fire end")
      );


  SequentialCommandGroup angleFire = new SequentialCommandGroup(
        Toolkit.sout("Fire init"),
          new RunArmClosedLoop(m_arm, ArmConstants.kAnglePos),
          new AccelerateShooter(m_shooter, ShooterConstants.kAngleSpeed),
        Toolkit.sout("shoot init"),
        new ParallelRaceGroup(
          new WaitCommand(ShooterConstants.kLaunchTime),
          new RunShooterEternal(m_shooter, ShooterConstants.kAngleSpeed, true),
          new Feed(m_intake)
        ),
        Toolkit.sout("Fire end")
      );

  SequentialCommandGroup backAmp = new SequentialCommandGroup(
        Toolkit.sout("AMP init"),
          new RunArmClosedLoop(m_arm, ArmConstants.kBackAmpPos),
          new AccelerateShooter(m_shooter, ShooterConstants.kAmpSpeed),
        Toolkit.sout("shoot init"),
        new ParallelRaceGroup(
          new WaitCommand(ShooterConstants.kLaunchTime),
          new RunShooterEternal(m_shooter, ShooterConstants.kAmpSpeed, true),
          new Feed(m_intake)
        ),
        Toolkit.sout("Amp end")
      );

  SequentialCommandGroup frontAmp = new SequentialCommandGroup(
        Toolkit.sout("AMP init"),
        new RunArmClosedLoop(m_arm, ArmConstants.kBackAmpPos),
        Toolkit.sout("shoot init"),
        new ParallelRaceGroup(
          new WaitCommand(ShooterConstants.kLaunchTime),
          new RunIntakeOpenLoop(m_intake, IntakeConstants.kReverseSpeed)
        ),
        Toolkit.sout("Amp end")
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
    NamedCommands.registerCommand("shootSubwoofer", new SequentialCommandGroup(
        Toolkit.sout("sFire init"),
          new RunArmClosedLoop(m_arm, ArmConstants.kSubwooferPos),
          new AccelerateShooter(m_shooter, ShooterConstants.kSubwooferSpeed),
        Toolkit.sout("shoot init"),
        new ParallelRaceGroup(
          new WaitCommand(ShooterConstants.kLaunchTime),
          new RunShooterEternal(m_shooter, ShooterConstants.kSubwooferSpeed, true),
          new Feed(m_intake)
        ),
        Toolkit.sout("sFire end")
      ));
    NamedCommands.registerCommand("shootpodium", new SequentialCommandGroup(
        Toolkit.sout("Fire init"),
          new RunArmClosedLoop(m_arm, ArmConstants.k3mPos),
          new AccelerateShooter(m_shooter, ShooterConstants.k3mSpeed),
        Toolkit.sout("shoot init"),
        new ParallelRaceGroup(
          new WaitCommand(ShooterConstants.kLaunchTime),
          new RunShooterEternal(m_shooter, ShooterConstants.k3mSpeed, true),
          new Feed(m_intake)
        ),
        Toolkit.sout("Fire end")
      ));
    NamedCommands.registerCommand("armInside", new RunArmClosedLoop(m_arm, ArmConstants.kStowPos));
    NamedCommands.registerCommand("homePort", new HomeClimber(m_portClimber));
    NamedCommands.registerCommand("homeStarboard", new HomeClimber(m_starboardClimber));
    NamedCommands.registerCommand("armDown", new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos));
    NamedCommands.registerCommand("drivetrainStop", new RunCommand(
      () -> m_robotDrive.drive(
        0, 0, 0, false, true), m_robotDrive));

    // Another option that allows you to specify the default auto by its name
    autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    dashboard = new Dashboard(m_robotDrive, m_arm, m_intake, m_shooter, m_portClimber, m_starboardClimber, autoChooser);
    Shuffleboard.getTab("match").add(autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(

      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController.getX()*DriverConstants.kDefaultSpeed, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getY()*DriverConstants.kDefaultSpeed, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getTwist()*DriverConstants.kYawSpeed, OIConstants.kDriveDeadband),
          true, true), m_robotDrive));

    m_driverController.button(DriverConstants.kAutoAmp). whileTrue(
      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController.getX()*DriverConstants.kDefaultSpeed, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getY()*DriverConstants.kDefaultSpeed, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getTwist()*DriverConstants.kYawSpeed, OIConstants.kDriveDeadband),
          true, true), m_robotDrive));

    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    m_operatorController.leftStick().or(m_operatorController.rightStick()).toggleOnTrue(
    new RunCommand(
      () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_operatorController.getLeftY()*OperatorConstants.kManoeuvreSpeed, OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_operatorController.getLeftX()*OperatorConstants.kManoeuvreSpeed, OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_operatorController.getRightX()*OperatorConstants.kManoeuvreSpeed, OIConstants.kDriveDeadband),
        true, true), m_robotDrive));

    m_driverController.button(DriverConstants.kTurbo).whileTrue(
      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getTwist(), OIConstants.kDriveDeadband),
          true, true), m_robotDrive));
    
    m_operatorController.start().whileTrue(
      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_operatorController.getLeftY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_operatorController.getLeftX(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_operatorController.getRightX()*DriverConstants.kYawSpeed, OIConstants.kDriveDeadband),
          false, true), m_robotDrive));

    // Subsystem Default Commands
    //m_intake.setDefaultCommand(new HoldIntake(m_intake));
    m_arm.setDefaultCommand(new SequentialCommandGroup(
      new RunArmClosedLoop(m_arm, ArmConstants.kDefaultPos),
      new HoldArm(m_arm)));
    m_shooter.setDefaultCommand(new RunShooterAtVelocity(m_shooter, ShooterConstants.kIdleSpeed, false));
    m_portClimber.setDefaultCommand(new HoldClimber(m_portClimber));
    m_starboardClimber.setDefaultCommand(new HoldClimber(m_starboardClimber));

    Shuffleboard.getTab("match").addBoolean("override", () -> override);
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
    m_driverController.button(DriverConstants.kStowArm).toggleOnTrue(new SequentialCommandGroup(
      new RunArmClosedLoop(m_arm, ArmConstants.kStowPos),
      new HoldArm(m_arm)
    ));
    m_driverController.button(DriverConstants.kAutoIntake).and(m_driverController.button(DriverConstants.kIntake)).onTrue(
      new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)
    );
    m_driverController.button(DriverConstants.kIntake).and(m_driverController.button(DriverConstants.kAutoIntake)).onTrue(
      new InstantCommand(() -> System.out.println("reset gyro"))
    );
    m_driverController.button(DriverConstants.kSelfDestruct).onTrue(
      new InstantCommand(() -> System.out.println("¡KABOOM!"))
    );

    m_operatorController.start().onTrue(new InstantCommand(() -> override = true));
    m_operatorController.start().onFalse(new InstantCommand(() -> override = false));
    m_operatorController.start().and(m_operatorController.povUp()).whileTrue(new RunArmOpenLoop(m_arm, ArmConstants.kManualSpeed));
    m_operatorController.start().and(m_operatorController.povDown()).whileTrue(new RunArmOpenLoop(m_arm, -ArmConstants.kManualSpeed));
    m_operatorController.start().and(m_operatorController.povRight()).whileTrue(new ForceFeed(m_intake, m_shooter));
    m_operatorController.start().and(m_operatorController.povLeft()).whileTrue(new ForceReverse(m_intake, m_shooter));

    m_operatorController.back().whileTrue(homeClimbers);
    m_operatorController.rightBumper().whileTrue(new RunClimberNormalLaw(m_starboardClimber, true));
    m_operatorController.rightTrigger().whileTrue(new RunClimberNormalLaw(m_starboardClimber, false));
    m_operatorController.leftBumper().whileTrue(new RunClimberNormalLaw(m_portClimber, true));
    m_operatorController.leftTrigger().whileTrue(new RunClimberNormalLaw(m_portClimber, false));

    m_operatorController.a().whileTrue(backAmp);
    m_operatorController.b().whileTrue(shuttleFire);
    m_operatorController.y().whileTrue(subwooferFire);
    m_operatorController.x().whileTrue(new ParallelCommandGroup(
      new IntakeNoteAutomatic(m_intake),
      new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos)));
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
