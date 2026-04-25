package frc.robot.subsystems.Mechanism.Hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Mechanism.DeviceIDs;

public class HoodIOHardware extends SubsystemBase implements HoodIO {

    private final double kMinPosition = 0.01;
    private final double kMaxPosition = 0.77;

    private final Servo leftServo;
    private final Servo rightServo;

    private double currentPosition = 0.5;

    public HoodIOHardware() {
        leftServo = new Servo(DeviceIDs.Hood.LEFT_SERVO_PWM_CHANNEL);
        rightServo = new Servo(DeviceIDs.Hood.RIGHT_SERVO_PWM_CHANNEL);
        leftServo.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);
        rightServo.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);
        setPosition(currentPosition);
        SmartDashboard.putData(this);
    }

    @Override
    public void setPosition(double position) {
        if (position < kMinPosition) {
            position = kMinPosition;
        } else if (position > kMaxPosition) {
            position = kMaxPosition;
        }

        final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);
        leftServo.set(clampedPosition);
        rightServo.set(clampedPosition);
    }
}
