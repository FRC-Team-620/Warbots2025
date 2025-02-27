package org.jmhsrobotics.frc2025.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private ClimberIO climberIO;
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private double outputDutyCycle = 0;

  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(inputs);
    Logger.recordOutput("Climber/Position Degrees", inputs.positionDegrees);
    Logger.recordOutput("Climber/Output Speed Duty Cycle", this.outputDutyCycle);
  }

  public void set(double climberSpeedDutyCycle) {
    this.outputDutyCycle = climberSpeedDutyCycle;
    climberIO.set(climberSpeedDutyCycle);
  }

  public double getClimberPositionDegrees() {
    return inputs.positionDegrees;
  }

  public void setBrakeMode(boolean enable) {
    climberIO.setBrakeMode(enable);
  }
}
