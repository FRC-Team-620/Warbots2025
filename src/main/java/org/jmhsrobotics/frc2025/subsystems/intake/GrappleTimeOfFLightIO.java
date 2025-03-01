package org.jmhsrobotics.frc2025.subsystems.intake;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
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
      algaeSensor.setTimingBudget(TimingBudget.TIMING_BUDGET_50MS);
      coralSensor.setTimingBudget(TimingBudget.TIMING_BUDGET_50MS);
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
      inputs.coralMeasurementOutOfBounds =
          coralMeasure.status == LaserCan.LASERCAN_STATUS_OUT_OF_BOUNDS;
      inputs.coralMeasurementIsValid =
          coralMeasure.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    } else {
      inputs.coralDistance = 100; // TODO: include a better missing measure value
    }

    var algaeMeasure = algaeSensor.getMeasurement();
    if (coralMeasure != null) {
      inputs.algaeDistance = algaeMeasure.distance_mm;
      inputs.algaeMeasurementOutOfBounds =
          algaeMeasure.status == LaserCan.LASERCAN_STATUS_OUT_OF_BOUNDS;
      inputs.algaeMeasurementIsValid =
          algaeMeasure.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;

    } else {
      inputs.algaeDistance = 100; // TODO: include a better missing measure value
    }
  }
}
