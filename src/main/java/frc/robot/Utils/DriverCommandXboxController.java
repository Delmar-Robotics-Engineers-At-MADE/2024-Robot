// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class DriverCommandXboxController extends CommandXboxController {

    private static final double kDeadband = 0.05;

    public DriverCommandXboxController(int port) {
        super(port);
    }

    public Trigger intake() {
        return leftBumper().or(rightBumper());
    }

    public Trigger gyroReset() {
        return intake().and(y());
    }

    public Trigger autointake() {
        return y();
    }

    public Trigger shuttle() {
        return b();
    }

    public Trigger backAmp() {
        return a();
    }

    public Trigger xMode() {
        return x();
    }

    public Trigger override() {
        return start();
    }

    public double getYaw() {
        if(Toolkit.isInTolarance(getRightX(), 0.0, kDeadband)) {
            return Toolkit.manageTriggers(getLeftTriggerAxis(), getRightTriggerAxis(), kDeadband);
        }
        else {
            return Toolkit.stickAugments(getRightX(), kDeadband);
        }
    }

    @Override
    public double getLeftTriggerAxis() {
        return (1/super.getLeftTriggerAxis())/10;
    }

    @Override
    public double getRightTriggerAxis() {
        return (1/super.getRightTriggerAxis())/10;
    }

    @Override
    public double getLeftX() {
        return Toolkit.stickAugments(super.getLeftX(), kDeadband);
    }

    @Override
    public double getLeftY() {
        return Toolkit.stickAugments(super.getLeftY(), kDeadband);
    }

    public Trigger slow() {
        return leftStick().or(rightStick());
    }
    


}

