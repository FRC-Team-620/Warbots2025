package org.jmhsrobotics.frc2025.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimTimeOfFlightIO implements TimeOfFLightIO {
  public SimTimeOfFlightIO() {
    SmartDashboard.putBoolean("sim/hasCoral", false);
    SmartDashboard.putBoolean("sim/hasBall", false);
  }

  @Override
  public void updateInputs(TimeOfFLightIOInputs inputs) {

    inputs.algaeMeasurementIsValid = true;
    inputs.coralMeasurementIsValid = true;
    // distance is in millimeters
    if (SmartDashboard.getBoolean("sim/hasCoral", false)) {
      inputs.coralDistance = 0;
    } else {
      inputs.coralDistance = 300; // TODO: replace with actual sensor reading
    }
    // distance is in millimeterss
    if (SmartDashboard.getBoolean("sim/hasBall", false)) {
      inputs.algaeDistance = 0;
    } else {
      inputs.algaeDistance = 600; // TODO: replace with actual sensor reading
    }
    // inputs.coralDistance = 60; // TODO: replace with actual sensor reading
    // inputs.algaeDistance = 60; // TODO: replace with actual sensor reading
  }
}
