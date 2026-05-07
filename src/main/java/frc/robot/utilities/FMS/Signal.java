package frc.robot.utilities.FMS;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Signal {
    public String gameData;
    public boolean CanGetPoint;

    private final double MatchTime = 140.0;
    private final double TRANSITION = 10.0;
    private final double Round = 25.0;

    public Signal() {
    }

    public char getAllianceChar() {
        char alliance;

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            alliance = 'R';
        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
            alliance = 'B';
        } else {
            alliance = 'N';
        }

        return alliance;
    }

    public char getInactive() {
        gameData = DriverStation.getGameSpecificMessage();

        char allience = 'N';

        if (gameData != null && gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    allience = 'B';
                    break;
                case 'R':
                    allience = 'R';
                    break;
                default:
                    allience = 'N';
                    break;
            }
        }
        return allience;
    }

    public boolean isInactive() {
        return getAllianceChar() == getInactive();
    }

    public boolean Active() {
        if (DriverStation.isAutonomous()) {
            return true;
        }
        if (isTRANSITION()) {
            return true;
        } else if (isInRound(1)) {
            if (!isInactive())
                return true;
            else
                return false;
        } else if (isInRound(2)) {
            if (isInactive())
                return true;
            else
                return false;
        } else if (isInRound(3)) {
            if (!isInactive())
                return true;
            else
                return false;
        } else if (isInRound(4)) {
            if (isInactive())
                return true;
            else
                return false;
        } else {
            return true;
        }
    }

    public boolean isInRound(int Round) {
        if (DriverStation.getMatchTime() <= MatchTime - TRANSITION - (Round - 1 * this.Round) &&
                DriverStation.getMatchTime() >= MatchTime - TRANSITION - (Round * this.Round)) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isTRANSITION() {
        if (DriverStation.getMatchTime() >= MatchTime - TRANSITION) {
            return true;
        } else {
            return false;
        }
    }
}