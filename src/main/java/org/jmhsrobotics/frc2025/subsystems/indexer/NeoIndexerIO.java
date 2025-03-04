package org.jmhsrobotics.frc2025.subsystems.indexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.util.SparkUtil;

public class NeoIndexerIO implements IndexerIO {
  private SparkMax motor = new SparkMax(Constants.CAN.kIndexerMotorID, MotorType.kBrushless);
  private SparkMaxConfig motorConfig = new SparkMaxConfig();

  private RelativeEncoder relativeEncoder = motor.getEncoder();

  private SparkClosedLoopController pidController;
  private double setPointDegrees;

  public NeoIndexerIO() {
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(8)
        .voltageCompensation(12)
        .inverted(false)
        .closedLoop
        .outputRange(-0.5, 0.5)
        .pid(
            Constants.IndexerConstants.kP,
            Constants.IndexerConstants.kI,
            Constants.IndexerConstants.kD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    motorConfig.encoder.positionConversionFactor(Constants.IndexerConstants.kConversionFactor);

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    pidController = motor.getClosedLoopController();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    SparkUtil.ifOk(
        motor, motor.getEncoder()::getPosition, (value) -> inputs.positionDegrees = value);
    SparkUtil.ifOk(motor, relativeEncoder::getVelocity, (value) -> inputs.motorRPM = value);
    SparkUtil.ifOk(motor, motor::getOutputCurrent, (value) -> inputs.motorAmps = value);

    pidController.setReference(setPointDegrees, ControlType.kPosition);
  }

  @Override
  public void setPositionDegrees(double setPointDegrees) {
    this.setPointDegrees = setPointDegrees;
  }
}
