package org.jmhsrobotics.frc2025.subsystems.climber.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double motorAmps;
    public double positionDegrees;
    public double motorRPM;
    public double setPointDegrees;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setPositionDegrees(double setPointDegrees) {}
}
