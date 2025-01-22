package org.jmhsrobotics.frc2025.subsystems.elevator;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.jmhsrobotics.frc2025.Constants;

public class VortexElevatorIO implements ElevatorIO {
  private SparkFlex vortexLeft = new SparkFlex(Constants.elevatorMotor1ID, MotorType.kBrushless);
  private SparkFlex vortexRight = new SparkFlex(Constants.elevatorMotor2ID, MotorType.kBrushless);
  private SparkFlexConfig vortexLeftConfig;
  private SparkFlexConfig vortexRightConfig;

  public VortexElevatorIO() {
    vortexLeftConfig = new SparkFlexConfig();
    vortexLeftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60)
        .voltageCompensation(12)
        .inverted(true);

    vortexRightConfig = new SparkFlexConfig();
    vortexRightConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60)
        .voltageCompensation(12)
        .inverted(false);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.motorAmps = new double[] {vortexLeft.getOutputCurrent(), vortexRight.getOutputCurrent()};
  }

  @Override
  public void setSetpoint(double PositionMeters) {
    // TODO Auto-generated method stub
  }
}
