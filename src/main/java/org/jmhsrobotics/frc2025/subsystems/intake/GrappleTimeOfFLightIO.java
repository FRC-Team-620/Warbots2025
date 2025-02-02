package org.jmhsrobotics.frc2025.subsystems.intake;

import au.grapplerobotics.LaserCan;
import org.jmhsrobotics.frc2025.Constants;

public class GrappleTimeOfFLightIO implements TimeOfFLightIO {
  private LaserCan coralSensor;
  private LaserCan algaeSensor;

  public GrappleTimeOfFLightIO() {
    coralSensor = new LaserCan(Constants.IntakeConstants.kCoralSensorId);
    algaeSensor = new LaserCan(Constants.IntakeConstants.kAlgaeSensorId);
  }

  @Override
  public void updateInputs(TimeOfFLightIOInputs inputs) {
    // distance is in millimeters
    inputs.coralDistance = coralSensor.getMeasurement().distance_mm;
    inputs.algaeDistance = algaeSensor.getMeasurement().distance_mm;
  }
}
