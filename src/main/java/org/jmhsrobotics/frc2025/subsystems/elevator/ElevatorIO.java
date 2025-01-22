package org.jmhsrobotics.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorInputs {
    public double positionMeters;
    public double velocityMPS;
    public double[] motorAmps;
  }

  public default void updateInputs(ElevatorInputs inputs) {}

  public void setSetpoint(double PositionMeters);
}
