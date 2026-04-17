package frc.robot.utilities.RobotState;

import frc.robot.utilities.RobotState.RobotState.AllianceState;
import frc.robot.utilities.RobotState.RobotState.LocationState;

public class FieldState {
    
    public final AllianceState allianceState;
    public final LocationState locationState;

    public FieldState(AllianceState alliance, LocationState location) {
        this.allianceState = alliance;
        this.locationState = location;
    }
}
