// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LEDConstants;

public class Blinkin extends SubsystemBase {
  private static Blinkin instance = null;

  public static Blinkin getInstance() {
    if (instance == null) instance = new Blinkin();

    return instance;
  }
  private final Spark one;
  private final Spark two;
  /** Creates a new Blinkin. */
  public Blinkin() {
    one = new Spark(0);
    two = new Spark(1);
  }

  public Command setOne(double colour) {
    return runOnce(()-> one.set(colour));
  }

  public Command setTwo(double colour) {
    return runOnce(()-> two.set(colour));
  }

  public Command set(double colour) {
    return setOne(colour);
  }

  public void setDefault() {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      if(alliance.get() == Alliance.Red) {
        super.setDefaultCommand(red());
      }
      else {
        super.setDefaultCommand(blue());
      }
  }

  public Command indCapture() {
    return new SequentialCommandGroup(
      set(LEDConstants.green),
      new WaitCommand(0.2),
      set(LEDConstants.grey),
      new WaitCommand(0.2),
      set(LEDConstants.green),
      new WaitCommand(0.2)
    );
  }

  public Command purple() {
    return set(LEDConstants.purple);
  }

  public Command red() {
    return set(LEDConstants.red);
  }

  public Command blue() {
    return set(LEDConstants.blue);
  }
}
