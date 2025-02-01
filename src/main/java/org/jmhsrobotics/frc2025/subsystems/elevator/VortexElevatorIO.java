package org.jmhsrobotics.frc2025.subsystems.elevator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.util.SparkUtil;

public class VortexElevatorIO implements ElevatorIO {
  private SparkFlex vortexLeft =
      new SparkFlex(Constants.ElevatorConstants.kMotorLeftId, MotorType.kBrushless);
  private SparkFlex vortexRight =
      new SparkFlex(Constants.ElevatorConstants.kMotorRightId, MotorType.kBrushless);
  private SparkFlexConfig vortexLeftConfig;
  private SparkFlexConfig vortexRightConfig;
  private SparkClosedLoopController pidController;

  private double goalMeters = 0;

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
        .inverted(false)
        .follow(vortexLeft);

    SparkUtil.tryUntilOk(
        vortexLeft,
        5,
        () ->
            vortexLeft.configure(
                vortexLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(
        vortexRight,
        5,
        () ->
            vortexRight.configure(
                vortexRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    pidController = vortexLeft.getClosedLoopController();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.motorAmps = new double[] {vortexLeft.getOutputCurrent(), vortexRight.getOutputCurrent()};
    inputs.motorRPM =
        new double[] {
          vortexLeft.getEncoder().getVelocity(), vortexRight.getEncoder().getVelocity()
        };

    inputs.motorPositionMeters =
        new double[] {
          vortexLeft.getAbsoluteEncoder().getPosition(),
          vortexRight.getAbsoluteEncoder().getPosition()
        };

    // TODO
    inputs.positionMeters = vortexLeft.getEncoder().getPosition();

    pidController.setReference(this.goalMeters, ControlType.kPosition);
  }

  public void setPositionMeters(double positionMeters) {
    this.goalMeters = positionMeters;
  }
}
