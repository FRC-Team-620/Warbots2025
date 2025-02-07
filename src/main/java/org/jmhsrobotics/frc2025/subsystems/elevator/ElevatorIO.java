package org.jmhsrobotics.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    // index[0] for arrays is the left motor
    public double[] motorPositionMeters;
    public double[] motorAmps;
    public double[] motorRPM;
    public double[] motorVolts;
    public double velocityMPS;
    public double heightMeters;
    public boolean isOpenLoop = true;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setPositionMeters(double heightMeters) {}

  public default void setVoltage(double voltage) {}

  public default void setZero() {}

}
