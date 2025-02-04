package org.jmhsrobotics.frc2025.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

  private TimeOfFLightIO timeOfFLightIO;
  private TimeOfFLightIOInputsAutoLogged sensorInputs = new TimeOfFLightIOInputsAutoLogged();

  private int mode;
  private boolean override;

  public Intake(IntakeIO intakeIO, TimeOfFLightIO timeOfFLightIO) {
    this.intakeIO = intakeIO;
    this.timeOfFLightIO = timeOfFLightIO;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    timeOfFLightIO.updateInputs(sensorInputs);
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
    if (sensorInputs.algaeDistance <= 50) return 0;
    else if (sensorInputs.coralDistance <= 30) return 2;
    return 1;
  }

  public void setMode(int increment) {
    this.override = true;
    this.mode += increment;
    if (this.mode < 0) {
      this.mode = 0;
    } else if (this.mode > 2) {
      this.mode = 2;
    }
  }

  public void set(double speedDutyCycle) {
    intakeIO.set(speedDutyCycle);
  }
}
