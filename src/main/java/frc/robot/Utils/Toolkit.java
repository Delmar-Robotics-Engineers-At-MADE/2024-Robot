package frc.robot.Utils;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AprilTags;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.States;

public final class Toolkit {
    
    public static boolean isInTolarance(double input, double target, double tolerance) {
        double upLim = target + tolerance;
        double downLim = target - tolerance;
        if(downLim <= input && input <= upLim) {
            return true;
        }
        else {
            return false;
        }
    }

    public static int getFiducialID(AprilTags tag) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        int result = 0;
        if(alliance.get() == Alliance.Red) {
            switch (tag) {
                case SUBSTATION_CLOSE:
                    result = 10;
                    break;
                case SUBSTATION_FAR:
                    result = 9;
                    break;
                case SPEAKER_CENTRE:
                    result = 4;
                    break;
                case SPEAKER_OFFSET:
                    result = 3;
                    break;
                case AMP:
                    result = 5;
                    break;
                case STAGE_LEFT:
                    result = 11;
                    break;
                case STAGE_RIGHT:
                    result = 12;
                    break;
                case STAGE_FAR:
                    result = 13;
                    break;
            }
        }
        else {
            switch (tag) {
                case SUBSTATION_CLOSE:
                    result = 1;
                    break;
                case SUBSTATION_FAR:
                    result = 2;
                    break;
                case SPEAKER_CENTRE:
                    result = 7;
                    break;
                case SPEAKER_OFFSET:
                    result = 8;
                    break;
                case AMP:
                    result = 6;
                    break;
                case STAGE_LEFT:
                    result = 15;
                    break;
                case STAGE_RIGHT:
                    result = 16;
                    break;
                case STAGE_FAR:
                    result = 14;
                    break;
            }
        }
        return result;
    }

    public static boolean invert(boolean input) {
        return !input;
    }

    public static double stickAugments(double input, double deadband) {
        return -MathUtil.applyDeadband(input, deadband);
    }

    public static Command sout(String input) {
        return new InstantCommand(() -> System.out.println(input));
    }

    public static double manageTriggers(double left, double right) {
        return -left + right;
    }

    public static int convertCardinalDirections(int povAngleDeg) {
        // change d-pad values for left and right to specified angle
        if (povAngleDeg == 270) {
          povAngleDeg += 77;
        } else if (povAngleDeg == 90) {
          povAngleDeg -= 77;
        }
        // targetHeadingDegrees is counterclockwise so need to flip povAngle
        povAngleDeg = 360 - povAngleDeg;
        return povAngleDeg;
    }

    public static double compensate(double value) {
        return value*ShooterConstants.kCompenstion;
    }

    // public static double[] getStateConstants(States state) {
    //     switch (state) {
    //         case INTAKE:
    //             double result[] = {ArmConstants.kIntakePos, IntakeConstants.kIntakeSpeed};
    //             break;
            
    //         default:
    //             break;
    //     }
    //     return result;
    // }

}
