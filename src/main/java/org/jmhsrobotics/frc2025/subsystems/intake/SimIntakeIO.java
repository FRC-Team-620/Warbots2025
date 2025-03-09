package org.jmhsrobotics.frc2025.subsystems.intake;

import org.littletonrobotics.junction.Logger;

public class SimIntakeIO implements IntakeIO {

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorAmps = 0;
    inputs.motorRPM = 0;
  }

  @Override
  public void set(double speedDutyCycle) {
    // just record the value so its visibe in AScope
    Logger.recordOutput("Intake/speedDutyCycle", speedDutyCycle);
  }
}
