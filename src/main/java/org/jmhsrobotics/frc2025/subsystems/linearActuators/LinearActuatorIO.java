package org.jmhsrobotics.frc2025.subsystems.linearActuators;

import org.littletonrobotics.junction.AutoLog;

public interface LinearActuatorIO {
  @AutoLog
  public static class LinearActuatorIOInputs {
    public double motorRPM;
    public double motorAmps;
  }

  public default void updateInputs(LinearActuatorIOInputs inputs) {}

  public default void set(double speedDutyCycle) {}
}
