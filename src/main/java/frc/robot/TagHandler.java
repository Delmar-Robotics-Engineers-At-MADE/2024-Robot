package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AprilTags;

public class TagHandler {

    private final Optional<Alliance> alliance;
    public TagHandler() {
        alliance = DriverStation.getAlliance();
    }

    public int getTag(AprilTags tag) {
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
}
