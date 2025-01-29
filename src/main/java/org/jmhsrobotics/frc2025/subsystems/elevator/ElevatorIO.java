package org.jmhsrobotics.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double[] motorPositionMeters;
    public double velocityMPS;
    public double[] motorRPM;
    public double[] motorAmps;
    public double positionMeters;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setPositionMeters(double positionMeters) {}
}
