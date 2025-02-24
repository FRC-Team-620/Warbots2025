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

  // private DiminishingAverageHandler coralAverageHandler = new
  // DiminishingAverageHandler(Constants.IntakeConstants.kCoralAverageHandlerWeight);
  // private DiminishingAverageHandler algaeAverageHandler = new
  // DiminishingAverageHandler(Constants.IntakeConstants.kAlgaeAverageHandlerWeight);

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

    // coralAverageHandler.feed(sensorInputs.coralDistance);
    // algaeAverageHandler.feed(sensorInputs.algaeDistance);
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
    if (this.getAveragedAlgaeDistance() <= 30 && this.getAveragedAlgaeDistance() != 0) return 1;
    else if (this.getAveragedCoralDistance() <= 20 && this.getAveragedCoralDistance() != 0)
      return 3;
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
   * gets the instantaneous coral distance
   *
   * @return
   */
  public double getCoralDistance() {
    return sensorInputs.coralDistance;
  }

  /**
   * gets the instantaneous coral distance
   *
   * @return
   */
  public double getAlgaeDistance() {
    return sensorInputs.algaeDistance;
  }

  /**
   * Returns the average coral distance over the last several measurements, removing noise
   *
   * @return
   */
  public double getAveragedCoralDistance() {
    double totalDistance = 0;
    for (int distance : sensorInputs.pastCoralDistance) {
      totalDistance += distance;
    }
    return totalDistance / sensorInputs.pastCoralDistance.length;

    // return coralAverageHandler.get();
  }

  /**
   * Returns the average algae distance over the last several measurements, removing noise
   *
   * @return
   */
  public double getAveragedAlgaeDistance() {
    double totalDistance = 0;
    for (int distance : sensorInputs.pastAlgaeDistance) {
      totalDistance += distance;
    }
    return totalDistance / sensorInputs.pastAlgaeDistance.length;

    // return algaeAverageHandler.get();
  }
}
