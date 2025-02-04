package org.jmhsrobotics.frc2025.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

  private TimeOfFLightIO timeOfFLightIO;
  private TimeOfFLightIOInputsAutoLogged sensorInputs = new TimeOfFLightIOInputsAutoLogged();

  public Intake(IntakeIO intakeIO, TimeOfFLightIO timeOfFLightIO) {
    this.intakeIO = intakeIO;
    this.timeOfFLightIO = timeOfFLightIO;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    timeOfFLightIO.updateInputs(sensorInputs);
  }

  // Method to determine and return current control mode: Algae, Coral, or Search
  // Algae will be mode 0, Search will be 1, Coral will be 2
  // Possibly return an enum containing the correct setpoint values for each mode?
  public int getMode() {
    if (sensorInputs.algaeDistance <= 50) return 0;
    else if (sensorInputs.coralDistance <= 30) return 2;
    return 1;
  }

  public void set(double speedDutyCycle) {
    intakeIO.set(speedDutyCycle);
  }
}
