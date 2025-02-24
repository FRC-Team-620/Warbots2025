package org.jmhsrobotics.frc2025.subsystems.intake;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import org.jmhsrobotics.frc2025.Constants;

public class GrappleTimeOfFLightIO implements TimeOfFLightIO {
  private LaserCan coralSensor;
  private LaserCan algaeSensor;

  public GrappleTimeOfFLightIO() {
    coralSensor = new LaserCan(Constants.CAN.kCoralSensorID);
    algaeSensor = new LaserCan(Constants.CAN.kAlgaeSensorID);

    try {
      coralSensor.setRangingMode(RangingMode.SHORT);
      algaeSensor.setRangingMode(RangingMode.SHORT);
      // TODO: set timing budget for sensors and find what is ideal
      // coralSensor.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration Failed: " + e);
    }
  }

  @Override
  public void updateInputs(TimeOfFLightIOInputs inputs) {
    // distance is in millimeters
    var coralMeasure = coralSensor.getMeasurement();
    if (coralMeasure != null) {
      inputs.coralDistance = coralMeasure.distance_mm;
    } else {
      inputs.coralDistance = 100; // TODO: include a better missing measure value
    }

    var algaeMeasure = algaeSensor.getMeasurement();
    if (coralMeasure != null) {
      inputs.algaeDistance = algaeMeasure.distance_mm;
    } else {
      inputs.algaeDistance = 100; // TODO: include a better missing measure value
    }

    // adds the current sensor value to the array, and moves all other values back by one index
    for (int i = inputs.pastCoralDistance.length - 1; i > 0; i--) {
      inputs.pastCoralDistance[i] = inputs.pastCoralDistance[i - 1];
    }
    inputs.pastCoralDistance[0] = coralMeasure.distance_mm;

    for (int i = inputs.pastAlgaeDistance.length - 1; i > 0; i--) {
      inputs.pastAlgaeDistance[i] = inputs.pastAlgaeDistance[i - 1];
    }
    inputs.pastAlgaeDistance[0] = algaeMeasure.distance_mm;
  }
}
