package org.jmhsrobotics.frc2025.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.Constants;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private IndexerIO indexerIO;
  private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private double setPointDegrees = Constants.IndexerConstants.kRotationDownDegrees;

  public Indexer(IndexerIO indexerIO) {
    this.indexerIO = indexerIO;
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(inputs);

    Logger.recordOutput("Indexer/Set Point", this.setPointDegrees);
    Logger.recordOutput("Indexer/Position Degrees", inputs.positionDegrees);
    Logger.recordOutput("Indexer/Current Amps", inputs.motorAmps);
  }

  public void setSetpoint(double setPointDegrees) {
    this.setPointDegrees = setPointDegrees;
    indexerIO.setPositionDegrees(setPointDegrees);
  }

  public double getPositionDegrees() {
    return inputs.positionDegrees;
  }

  public double getSetPointDegrees() {
    return this.setPointDegrees;
  }
}
