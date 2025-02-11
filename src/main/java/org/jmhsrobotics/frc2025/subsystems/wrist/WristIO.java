package org.jmhsrobotics.frc2025.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double positionDegrees;
    public double motorRPM;
    public double motorAmps;
    public double relativePositionDegrees;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setPositionDegrees(double angle) {}
}
