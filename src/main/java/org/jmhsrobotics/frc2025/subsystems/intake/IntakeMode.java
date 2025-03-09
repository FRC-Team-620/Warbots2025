package org.jmhsrobotics.frc2025.subsystems.intake;

public enum IntakeMode {
  ALGAE(1),
  IDLE(2),
  CORAL(3);

  private final int value;

  IntakeMode(int value) {
    this.value = value;
  }

  public int getValue() {
    return value;
  }

  public static IntakeMode fromInt(int value) {
    for (IntakeMode mode : IntakeMode.values()) {
      if (mode.getValue() == value) {
        return mode;
      }
    }
    return null;
  }
}
