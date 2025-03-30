package org.jmhsrobotics.frc2025.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private IndexerIO indexerIO;
  private IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO indexerIO) {
    this.indexerIO = indexerIO;
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(indexerInputs);

    Logger.recordOutput("Indexer/Indexer Current Amps", indexerInputs.motorAmps);
  }

  public void set(double speedDutyCycle) {
    indexerIO.set(speedDutyCycle);
  }

  public void setBrakeMode(boolean enable) {
    indexerIO.setBrakeMode(enable);
  }
}
