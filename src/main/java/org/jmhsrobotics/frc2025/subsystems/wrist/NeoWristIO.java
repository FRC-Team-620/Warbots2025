package org.jmhsrobotics.frc2025.subsystems.wrist;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.jmhsrobotics.frc2025.Constants;

public class NeoWristIO implements WristIO {
  private SparkMax motor = new SparkMax(Constants.WristConstants.kMotorId, MotorType.kBrushless);

  private SparkMaxConfig motorConfig;

  private SparkClosedLoopController pidController;

  private double goalDegrees;

  public NeoWristIO() {
    motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12)
        .inverted(false);

    pidController = motor.getClosedLoopController();
  }

  public void updateInputs(WristIOInputs inputs) {
    // getPosition() multiplied by 360 to convert from rotations to degrees
    inputs.positionDegrees = motor.getAbsoluteEncoder().getPosition() * 360;

    inputs.motorAmps = motor.getOutputCurrent();
    inputs.motorRPM = motor.getEncoder().getVelocity();

    pidController.setReference(goalDegrees, ControlType.kPosition);
  }

  public void setPositionDegrees(double positionDegrees) {
    this.goalDegrees = positionDegrees;
  }
}
