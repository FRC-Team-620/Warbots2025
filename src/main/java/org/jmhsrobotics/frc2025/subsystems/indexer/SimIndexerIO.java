package org.jmhsrobotics.frc2025.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

public class SimIndexerIO implements IndexerIO {
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.motorAmps = 0;
    inputs.motorRPM = 0;
  }

  @Override
  public void set(double speedDutyCycle) {
    // just record the value so its visibe in AScope
    Logger.recordOutput("Indexer/speedDutyCycle", speedDutyCycle);
  }
}
