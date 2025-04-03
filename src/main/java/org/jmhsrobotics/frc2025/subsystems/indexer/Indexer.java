package org.jmhsrobotics.frc2025.subsystems.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private IndexerIO indexerIO;
  private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private boolean coralInIndexer;
  private Timer accelerationTimer = new Timer();

  private boolean isAccelerating;

  private double goalSpeedRPM;

  private Debouncer debouncer = new Debouncer(0.15, DebounceType.kBoth);

  public Indexer(IndexerIO indexerIO) {
    this.indexerIO = indexerIO;
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(inputs);
    if (accelerationTimer.hasElapsed(1.15)) isAccelerating = false;
    else isAccelerating = true;
    coralInIndexer = debouncer.calculate(!this.atRPMGoal() && !isAccelerating);

    Logger.recordOutput("Indexer/Current Amps", inputs.motorAmps);
    Logger.recordOutput("Indexer/Speed RPM", inputs.motorRPM);
    Logger.recordOutput("Indexer/Goal RPM", this.goalSpeedRPM);
    Logger.recordOutput("Indexer/Output Duty Cycle", inputs.outputSpeedDutyCycle);
    Logger.recordOutput("Indexer/Temperature Celcius", inputs.motorTemperatureCelcius);
    Logger.recordOutput("Indexer/Coral In Indexer", this.coralInIndexer);
    Logger.recordOutput("Indexer/Is Indexer Accelerating", isAccelerating);
    Logger.recordOutput("Indexer/At RPM Goal", this.atRPMGoal());
  }

  public void set(double speedRPM) {
    if (speedRPM > 0) {
      accelerationTimer.start();
    } else {
      accelerationTimer.reset();
      accelerationTimer.stop();
    }
    goalSpeedRPM = speedRPM;
    indexerIO.set(speedRPM);
  }

  public void setBrakeMode(boolean enable) {
    indexerIO.setBrakeMode(enable);
  }

  public boolean hasCoral() {
    return this.coralInIndexer;
    // return false;
  }

  private boolean atRPMGoal() {
    return Math.abs(inputs.motorRPM - goalSpeedRPM) < 100;
  }
}
