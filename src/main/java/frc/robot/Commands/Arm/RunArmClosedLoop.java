// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Utils.Toolkit;
import frc.robot.subsystems.Arm;

public class RunArmClosedLoop extends Command {
  private Arm arm;
  private double setpoint;
  private boolean end;
  private TrapezoidProfile.State state;
  /** Creates a new RunArmClosedLoop. */
  public RunArmClosedLoop(Arm ar, double target) {
    arm = ar;
    setpoint = target;
    end = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("CMD init - running arm");
    end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Toolkit.isInTolarance(arm.getPos(), setpoint, ArmConstants.kTolearance)) {
      System.out.println("Arm At Setpoint");
      end = true;
    }
    else {
      state = new State(setpoint, 0);
      arm.runToPosition(state);
      end = false;
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
