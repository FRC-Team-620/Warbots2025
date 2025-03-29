package org.jmhsrobotics.frc2025.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double motorRPM;
    public double motorAmps;
    public double outputSpeedDutyCycle;
    public double motorTemperatureCelcius;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void set(double speedDutyCycle) {}

  public default void setBrakeMode(boolean enable) {}
}
