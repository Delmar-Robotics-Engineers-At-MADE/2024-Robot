// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Intake;

public class IntakeNoteAutomatic extends Command {
  private Intake intake;
  private boolean end;
  private boolean capture;
  private Blinkin blinkin;
  /** Creates a new IntakeNoteAutomatic. */
  public IntakeNoteAutomatic(Intake in/*, Blinkin blinkin*/) {
    intake = in;
    end = false;
    //this.blinkin = blinkin;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(in);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    end = false;
    capture = intake.isNote();
    intake.hiCurrent();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.isNote()) {
      capture = true;
      intake.halt();
      System.out.println("¡CAPTURE!");
      /*blinkin.indCapture();*/
      end = true;
    }
    else if(!capture) {
      intake.autoIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
