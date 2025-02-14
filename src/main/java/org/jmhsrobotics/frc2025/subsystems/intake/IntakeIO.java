package org.jmhsrobotics.frc2025.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double motorRPM;
    public double motorAmps;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void set(double speedDutyCycle) {}

  public default void setBrakeMode(boolean enable) {}
}
