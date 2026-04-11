package frc.robot.subsystems.Mechanism.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;

public interface ShooterIO {

    public void setVelocity(AngularVelocity velocityRPS);

    public AngularVelocity[] getVelocity();

    public boolean isAtSetpoint();

    public Command enable();

    public Command disable();

    public void configure(TalonFX motor, InvertedValue invertDirection);
}