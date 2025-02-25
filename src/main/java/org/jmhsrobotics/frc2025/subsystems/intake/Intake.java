package org.jmhsrobotics.frc2025.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

  private TimeOfFLightIO timeOfFLightIO;
  private TimeOfFLightIOInputsAutoLogged sensorInputs = new TimeOfFLightIOInputsAutoLogged();

  private int mode = 2;
  private boolean override = false;

  public Intake(IntakeIO intakeIO, TimeOfFLightIO timeOfFLightIO) {
    this.intakeIO = intakeIO;
    this.timeOfFLightIO = timeOfFLightIO;
  }

  @Override
  public void periodic() {
    this.mode = getMode();
    intakeIO.updateInputs(intakeInputs);
    timeOfFLightIO.updateInputs(sensorInputs);
    Logger.recordOutput("Current Control Mode", this.mode);
    Logger.recordOutput("Intake/Coral Sensor Distance", sensorInputs.coralDistance);
    Logger.recordOutput("Intake/Algae Sensor Distance", sensorInputs.algaeDistance);
  }

  /**
   * Determines the current control mode. If override is true returns mode, otherwise returns mode
   * based on ToF sensor inputs
   *
   * @return
   */
  public int getMode() {
    if (override) {
      return mode;
    }
    if (sensorInputs.algaeDistance <= 30 && sensorInputs.algaeDistance != 0) return 1;
    else if (sensorInputs.coralDistance <= 20 && sensorInputs.coralDistance != 0) return 3;
    return 2;
  }

  /**
   * Manual override to set control mode if sensors stop working during match. Override cannot be
   * reversed
   *
   * @param increment +1 to go from search to coral/algae to search, -1 for opposite
   */
  public void setMode(int increment) {
    this.override = true;
    this.mode += increment;
    if (this.mode < 1) {
      this.mode = 1;
    } else if (this.mode > 3) {
      this.mode = 3;
    }
  }

  /** Turns off manual override of control mode in the event of an accidental activation */
  public void unOverrideControlMode() {
    this.override = false;
  }

  public void set(double speedDutyCycle) {
    intakeIO.set(speedDutyCycle);
  }

  public void setBrakeMode(boolean enable) {
    intakeIO.setBrakeMode(enable);
  }

  public double getCoralDistance() {
    return sensorInputs.coralDistance;
  }

  public double getAlgaeDistance() {
    return sensorInputs.algaeDistance;
  }

  public boolean isControlModeOverridden() {
    return override;
  }
}
