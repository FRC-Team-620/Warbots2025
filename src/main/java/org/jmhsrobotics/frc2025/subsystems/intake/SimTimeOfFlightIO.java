package org.jmhsrobotics.frc2025.subsystems.intake;

public class SimTimeOfFlightIO implements TimeOfFLightIO {

  public SimTimeOfFlightIO() {}

  @Override
  public void updateInputs(TimeOfFLightIOInputs inputs) {
    // distance is in millimeters
    inputs.coralDistance = 60; // TODO: replace with actual sensor reading
    inputs.algaeDistance = 60; // TODO: replace with actual sensor reading

    // for (int i = 1; i < inputs.pastCoralDistance.length; i++) {
    //   inputs.pastCoralDistance[i] = inputs.pastCoralDistance[i - 1];
    // }
    // inputs.pastCoralDistance[0] = 60;

    // for (int i = 1; i < inputs.pastAlgaeDistance.length; i++) {
    //   inputs.pastAlgaeDistance[i] = inputs.pastAlgaeDistance[i - 1];
    // }
    // inputs.pastAlgaeDistance[0] = 60;
  }
}
