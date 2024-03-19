package frc.robot.Utils;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AprilTags;
import frc.robot.Constants.ShooterConstants;

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

    public static int[] getStageFiducialIDs() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.get() == Alliance.Red) {
            int[] result = {11,12,13};
            return result;
        } 
        else {
            int[] result = {15,16,14};
            return result;
        }
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

    public static double manageTriggers(double left, double right, double deadband) {
        return stickAugments(left - right, deadband);
    }

    public static int convertCardinalDirections(int povAngleDeg) {
        // change d-pad values for left and right to specified angle
        if (povAngleDeg == 270) {
          povAngleDeg += 30;
        } else if (povAngleDeg == 90) {
          povAngleDeg -= 30;
        }
        // targetHeadingDegrees is counterclockwise so need to flip povAngle
        povAngleDeg = 360 - povAngleDeg;
        return povAngleDeg;
    }

    public static double compensate(double value) {
        return value*ShooterConstants.kCompenstion;
    }

    public static double calculateDistanceOffset(double subsysOffset, double fieldOffset) {
        return subsysOffset + fieldOffset;
    }
}
