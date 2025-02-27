package org.jmhsrobotics.frc2025.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface TimeOfFLightIO {
  @AutoLog
  public static class TimeOfFLightIOInputs {
    // distance in millimeters
    public int coralDistance = -1;
    public int algaeDistance = -1;
    public boolean algaeMeasurementOutOfBounds = false;
    public boolean coralMeasurementOutOfBounds = false;
    public boolean algaeMeasurementIsValid = false;
    public boolean coralMeasurementIsValid = false;
  }

  public default void updateInputs(TimeOfFLightIOInputs inputs) {}
}
