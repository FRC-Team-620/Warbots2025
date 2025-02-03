package org.jmhsrobotics.frc2025.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface TimeOfFLightIO {
  @AutoLog
  public static class TimeOfFLightIOInputs {
    // distance in millimeters
    public int coralDistance;
    public int algaeDistance;
  }

  public default void updateInputs(TimeOfFLightIOInputs inputs) {}
}
