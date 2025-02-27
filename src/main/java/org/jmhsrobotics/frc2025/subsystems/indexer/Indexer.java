package org.jmhsrobotics.frc2025.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.Constants;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private IndexerIO indexerIO;
  private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private double setpointDegrees = 0;

  public Indexer(IndexerIO indexerIO) {
    this.indexerIO = indexerIO;
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(inputs);
    Logger.recordOutput("Indexer/Position Degrees", inputs.positionDegrees);
    Logger.recordOutput("Indexer/Setpoint Degrees", this.setpointDegrees);
  }

  public void setSetpoint(double setpointDegrees) {
    this.setpointDegrees = setpointDegrees;
    indexerIO.setPositionDegrees(setpointDegrees);
  }

  public double getSetpointDegrees() {
    return setpointDegrees;
  }

  public double getPositionDegrees() {
    return inputs.positionDegrees;
  }

  public boolean atGoal() {
    return Math.abs(this.setpointDegrees - inputs.positionDegrees)
        < Constants.IndexerConstants.kRotationToleranceDegrees;
  }
}
