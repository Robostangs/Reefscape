package frc.robot.Stable;

import frc.robot.Constants;
import frc.robot.Robot;

public class ScoringTargets {
    Constants.SwerveConstants.AutoConstants.ReefSides side;
    boolean isRight;
    /**
     * Constructor for ScoringTargets.
     *
     * @param side    The side of the reef where the scoring target is located.
     * @param isRight Indicates if the target is on the right side.
     */
    public ScoringTargets(Constants.SwerveConstants.AutoConstants.ReefSides side, boolean isRight){
        this.side = side;
        this.isRight = isRight;
    }

    public boolean isRight() {
        return isRight;
    }

    public int getAprilTag(){
        if (Robot.isRed()) {
            switch (side) {
                case Center1:
                    return 10;
                case Center2:
                    return 7;
                case Pro1:
                    return 11;
                case Pro2:
                    return 6;
                case Open1:
                    return 9;
                case Open2:
                    return 8;
                default:
                    return -1;
            }
        } else {
            switch (side) {
                case Center1:
                    return 18;
                case Center2:
                    return 21;
                case Pro1:
                    return 17;
                case Pro2:
                    return 22;
                case Open1:
                    return 19;
                case Open2:
                    return 20;
                default:
                    return -1; 
            }
        }
    }

}
