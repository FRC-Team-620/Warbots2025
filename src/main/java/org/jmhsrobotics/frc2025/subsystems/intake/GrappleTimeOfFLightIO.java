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
    var coralMeasure = coralSensor.getMeasurement();
    if (coralMeasure != null) {
      inputs.coralDistance = coralMeasure.distance_mm;
    } else {
      inputs.coralDistance = -1; // TODO: include a better missing measure value
    }

    var algaeMeasure = algaeSensor.getMeasurement();
    if (coralMeasure != null) {
      inputs.algaeDistance = algaeMeasure.distance_mm;
    } else {
      inputs.algaeDistance = -1; // TODO: include a better missing measure value
    }
  }
}
