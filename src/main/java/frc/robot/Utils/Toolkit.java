package frc.robot.Utils;

public class Toolkit {
    
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
}
