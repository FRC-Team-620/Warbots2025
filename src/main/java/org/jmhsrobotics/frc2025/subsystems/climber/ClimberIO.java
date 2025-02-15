package org.jmhsrobotics.frc2025.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double positionDegrees;
    public double motorAmps;
    public double motorRPM;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void set(double speedDutyCycle) {}

  public default void setBrakeMode(boolean enable) {}
}
