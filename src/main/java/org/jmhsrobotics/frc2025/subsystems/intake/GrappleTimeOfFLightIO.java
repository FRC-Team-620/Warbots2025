package org.jmhsrobotics.frc2025.subsystems.intake;

import au.grapplerobotics.LaserCan;
import org.jmhsrobotics.frc2025.Constants;

public class GrappleTimeOfFLightIO implements TimeOfFLightIO {
  private LaserCan coralSensor;
  private LaserCan algaeSensor;

  public GrappleTimeOfFLightIO() {
    coralSensor = new LaserCan(Constants.CAN.kCoralSensorID);
    algaeSensor = new LaserCan(Constants.CAN.kAlgaeSensorID);
  }

  @Override
  public void updateInputs(TimeOfFLightIOInputs inputs) {
    // distance is in millimeters

    // TODO: fix null pointer exception
    // inputs.coralDistance = coralSensor.getMeasurement().distance_mm;
    // inputs.algaeDistance = algaeSensor.getMeasurement().distance_mm;
  }
}
