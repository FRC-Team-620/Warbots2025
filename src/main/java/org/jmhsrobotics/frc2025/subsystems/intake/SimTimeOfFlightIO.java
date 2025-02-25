package org.jmhsrobotics.frc2025.subsystems.intake;

public class SimTimeOfFlightIO implements TimeOfFLightIO {

  public SimTimeOfFlightIO() {}

  @Override
  public void updateInputs(TimeOfFLightIOInputs inputs) {
    // distance is in millimeters
    inputs.coralDistance = 60; // TODO: replace with actual sensor reading
    inputs.algaeDistance = 60; // TODO: replace with actual sensor reading
  }
}
