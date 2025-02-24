package org.jmhsrobotics.frc2025.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private IndexerIO indexerIO;
  private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO indexerIO) {
    this.indexerIO = indexerIO;
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(inputs);
      Logger.recordOutput("Indexer/Position Degrees", inputs.positionDegrees);
      Logger.recordOutput("Indexer/Setpoint Degrees", inputs.setPointDegrees);
  }

  public void setSetpoint(double setpointDegrees){
    indexerIO.setPositionDegrees(setpointDegrees);
  }

  public double getPositionDegrees(){
    return inputs.positionDegrees;
  }
}
