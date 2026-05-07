package frc.robot.utilities.TargetCalculator;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class TargetMaps {

    public final InterpolatingDoubleTreeMap L_rollMap, M_rollMap, R_rollMap;
    public final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
    public final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

    public TargetMaps() {

        L_rollMap = new InterpolatingDoubleTreeMap();

        M_rollMap = new InterpolatingDoubleTreeMap();

        R_rollMap = new InterpolatingDoubleTreeMap();

        L_rollMap.put(1.52, 72.2);
        L_rollMap.put(1.97, 72.4);
        L_rollMap.put(2.26, 75d);
        L_rollMap.put(3.25, 79.8);
        L_rollMap.put(4.21, 91d);
        L_rollMap.put(5.1, 91.2);

        M_rollMap.put(1.52, 73.4);
        M_rollMap.put(1.97, 75d);
        M_rollMap.put(2.26, 77.8);
        M_rollMap.put(3.25, 88.6);
        M_rollMap.put(4.21, 89d);
        M_rollMap.put(5.1, 93.8);

        R_rollMap.put(1.52, 73.2);
        R_rollMap.put(1.97, 71.8);
        R_rollMap.put(2.26, 75.6);
        R_rollMap.put(3.25, 84.6);
        R_rollMap.put(4.21, 87d);
        R_rollMap.put(5.1, 92.8);


        hoodMap.put(1.52, 0.16);
        hoodMap.put(1.97, 0.16);
        hoodMap.put(2.26, 0.16);
        hoodMap.put(3.25, 0.24);
        hoodMap.put(4.21, 0.34);
        hoodMap.put(5.1, 0.39);

        timeOfFlightMap.put(1.0, 0.84);
        timeOfFlightMap.put(1.0, 0.98);
        timeOfFlightMap.put(2.0, 1.16);
        timeOfFlightMap.put(2.5, 1.16);
        timeOfFlightMap.put(3.0, 1.2);
        timeOfFlightMap.put(3.5, 1.21);
        timeOfFlightMap.put(4.0, 1.21);
        timeOfFlightMap.put(4.5, 1.32);
        timeOfFlightMap.put(5.0, 1.32);
    }

    // getter

    public double getLeftVelocity(Distance distance) {
        return this.L_rollMap.get(distance.in(Meter));
    }

    public double getMiddleVelocity(Distance distance) {
        return this.M_rollMap.get(distance.in(Meter));
    }

    public double getRightVelocity(Distance distance) {
        return this.R_rollMap.get(distance.in(Meter));
    }

    public double getHoodPosition(Distance distance) {
        return this.hoodMap.get(distance.in(Meter));
    }
}
