package frc.robot.utilities.TargetCalculator;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class TargetMaps {

    public final InterpolatingTreeMap<Double, AngularVelocity> rollMap;
    public final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
    public final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

    public TargetMaps() {

        rollMap = new InterpolatingTreeMap<>(
                InverseInterpolator.forDouble(),
                (start, end, t) -> {
                    double startVal = start.in(RotationsPerSecond);
                    double endVal = end.in(RotationsPerSecond);
                    double interpolated = MathUtil.interpolate(startVal, endVal, t);

                    return RotationsPerSecond.of(interpolated);
                });

        rollMap.put(1.0, RotationsPerSecond.of(32));
        rollMap.put(1.5, RotationsPerSecond.of(33));
        rollMap.put(2.0, RotationsPerSecond.of(34));
        rollMap.put(2.5, RotationsPerSecond.of(35));
        rollMap.put(3.0, RotationsPerSecond.of(36));
        rollMap.put(3.5, RotationsPerSecond.of(37));
        rollMap.put(4.0, RotationsPerSecond.of(38));
        rollMap.put(4.5, RotationsPerSecond.of(39));
        rollMap.put(5.0, RotationsPerSecond.of(40));

        hoodMap.put(1.0, 0.01);
        hoodMap.put(1.5, 0.12);
        hoodMap.put(2.0, 0.24);
        hoodMap.put(2.5, 0.35);
        hoodMap.put(3.0, 0.46);
        hoodMap.put(3.5, 0.57);
        hoodMap.put(4.0, 0.65);
        hoodMap.put(4.5, 0.73);
        hoodMap.put(5.0, 0.77);

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

    public AngularVelocity getFlyWheelVelocity(Distance distance) {
        return this.rollMap.get(distance.in(Meter));
    }

    public double getHoodPosition(Distance distance) {
        return this.hoodMap.get(distance.in(Meter));
    }
}
