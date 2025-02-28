package org.jmhsrobotics.frc2025.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

  private TimeOfFLightIO timeOfFLightIO;
  private TimeOfFLightIOInputsAutoLogged sensorInputs = new TimeOfFLightIOInputsAutoLogged();

  private int mode = 2;
  private boolean override = false;

  private Debouncer coralDebouncer =
      new Debouncer(Constants.IntakeConstants.kCoralDebounceTime, DebounceType.kBoth);
  private Debouncer algaeFallingDebouncer =
      new Debouncer(Constants.IntakeConstants.kAlgaeFallingDebounceTime, DebounceType.kFalling);
  private Debouncer algaeRisingDebouncer =
      new Debouncer(Constants.IntakeConstants.kAlgaeRisingDebounceTime, DebounceType.kRising);

  private boolean coralInIntake = false;
  private boolean algaeInIntake = false;

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
    Logger.recordOutput("Intake/Coral In Intake", coralInIntake);
    Logger.recordOutput("Intake/Algae In Intake", algaeInIntake);
    Logger.recordOutput("Intake/Coral Measurement Valid", sensorInputs.coralMeasurementIsValid);
    Logger.recordOutput("Intake/Algae Measurement Valid", sensorInputs.algaeMeasurementIsValid);
    Logger.recordOutput(
        "Intake/Coral Measurement Out Of Bounds", sensorInputs.coralMeasurementOutOfBounds);
    Logger.recordOutput(
        "Intake/Algae Measurement Out Of Bounds", sensorInputs.algaeMeasurementOutOfBounds);
  }

  /**
   * Determines the current control mode. If override is true returns mode, otherwise returns mode
   * based on ToF sensor inputs
   *
   * @return
   */
  public int getMode() {
    // determines if coral and algae are in the intake based on sensor inputs and debouncers
    coralInIntake =
        coralDebouncer.calculate(
            sensorInputs.coralDistance <= Constants.IntakeConstants.kCoralInIntakeDistanceMm
                && sensorInputs.coralMeasurementIsValid);
    algaeInIntake =
        algaeFallingDebouncer.calculate(
                sensorInputs.algaeDistance <= Constants.IntakeConstants.kAlgaeInIntakeDistanceMm
                    && sensorInputs.algaeMeasurementIsValid)
            && algaeRisingDebouncer.calculate(
                sensorInputs.algaeDistance <= Constants.IntakeConstants.kAlgaeInIntakeDistanceMm
                    && sensorInputs.algaeMeasurementIsValid);

    if (override) {
      return mode;
    }
    if (coralInIntake) {
      this.mode = 3;
      return this.mode;
    } else if (algaeInIntake) {
      this.mode = 1;
      return this.mode;
    }
    this.mode = 2;
    return this.mode;
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

  /**
   * sets the duty cycle speed of the intake motor
   *
   * @param speedDutyCycle
   */
  public void set(double speedDutyCycle) {
    intakeIO.set(speedDutyCycle);
  }

  public void setBrakeMode(boolean enable) {
    intakeIO.setBrakeMode(enable);
  }

  /**
   * returns the instantaneous coral distance
   *
   * @return
   */
  public double getCoralDistance() {
    if (sensorInputs.coralMeasurementIsValid) return sensorInputs.coralDistance;
    return 1000;
  }

  /**
   * returns the instantaneous coral distance
   *
   * @return
   */
  public double getAlgaeDistance() {
    if (sensorInputs.algaeMeasurementIsValid) return sensorInputs.algaeDistance;
    return 1000;
  }

  public boolean isCoralMeasureValid() {
    return sensorInputs.coralMeasurementIsValid;
  }

  public boolean isAlgaeMeasureValid() {
    return sensorInputs.algaeMeasurementIsValid;
  }

  public boolean isControlModeOverridden() {
    return override;
  }

  public boolean isCoralInIntake() {
    return this.coralInIntake;
  }

  public boolean isAlgaeInintake() {
    return this.algaeInIntake;
  }
}
