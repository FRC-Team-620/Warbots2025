package org.jmhsrobotics.frc2025.subsystems.wrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.util.SparkUtil;

public class NeoWristIO implements WristIO {
  private SparkMax motor = new SparkMax(Constants.CAN.kWristMotorID, MotorType.kBrushless);
  private AbsoluteEncoder encoder;
  private RelativeEncoder relativeEncoder;

  private SparkMaxConfig motorConfig = new SparkMaxConfig();
  private AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();

  private SparkClosedLoopController pidController;
  // P:0.02

  private double setPointDegrees;

  public NeoWristIO() {
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12)
        .inverted(false)
        .signals
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(20)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    motorConfig
        .closedLoop
        .pid(Constants.WristConstants.kP, Constants.WristConstants.kI, Constants.WristConstants.kD)
        .outputRange(-0.25, 0.25)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    encoderConfig.positionConversionFactor(360);
    encoder = motor.getAbsoluteEncoder();

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    pidController = motor.getClosedLoopController();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    SparkUtil.sparkStickyFault = false;
    SparkUtil.ifOk(motor, encoder::getPosition, (value) -> inputs.positionDegrees = value);

    SparkUtil.ifOk(motor, motor::getOutputCurrent, (value) -> inputs.motorAmps = value);
    SparkUtil.ifOk(motor, encoder::getVelocity, (value) -> inputs.motorRPM = value);

    pidController.setReference(setPointDegrees, ControlType.kPosition);

    inputs.relativePositionDegrees = motor.getEncoder().getPosition();
  }

  @Override
  public void setPositionDegrees(double positionDegrees) {
    this.setPointDegrees = positionDegrees;
  }
}
