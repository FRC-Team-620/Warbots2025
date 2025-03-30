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
  private boolean isAccelerating;
  private Timer accelerationTimer = new Timer();

  private double speedDutyCycle = 0;

  private Debouncer debouncer = new Debouncer(0.15, DebounceType.kBoth);

  public Indexer(IndexerIO indexerIO) {
    this.indexerIO = indexerIO;
  }

  @Override
  public void periodic() {
    if (accelerationTimer.hasElapsed(0.15)) {
      isAccelerating = false;
      accelerationTimer.reset();
    }

    indexerIO.updateInputs(inputs);
    coralInIndexer = debouncer.calculate(inputs.motorAmps > 2.5) && !isAccelerating;

    Logger.recordOutput("Indexer/Current Amps", inputs.motorAmps);
    Logger.recordOutput("Indexer/Speed RPM", inputs.motorRPM);
    Logger.recordOutput("Indexer/Output Duty Cycle", inputs.outputSpeedDutyCycle);
    Logger.recordOutput("Indexer/Temperature Celcius", inputs.motorTemperatureCelcius);
    Logger.recordOutput("Indexer/Coral In Indexer", this.coralInIndexer);
  }

  public void set(double speedDutyCycle) {
    if (speedDutyCycle != this.speedDutyCycle){
      isAccelerating = true;
      accelerationTimer.restart();
    }
    indexerIO.set(speedDutyCycle);
  }

  public void setBrakeMode(boolean enable) {
    indexerIO.setBrakeMode(enable);
  }
}
