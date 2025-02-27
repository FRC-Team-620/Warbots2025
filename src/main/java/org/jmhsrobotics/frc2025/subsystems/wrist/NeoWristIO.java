package org.jmhsrobotics.frc2025.subsystems.wrist;

import com.revrobotics.AbsoluteEncoder;
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
  private AbsoluteEncoder encoder = motor.getAbsoluteEncoder();

  private SparkMaxConfig motorConfig = new SparkMaxConfig();
  private AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();

  private SparkClosedLoopController pidController;
  // P:0.02

  private double setPointDegrees = Constants.WristConstants.kSafeAngleDegrees;

  public NeoWristIO() {
    encoderConfig.positionConversionFactor(360);

    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
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
        .outputRange(-0.5, 0.5)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    motorConfig.absoluteEncoder.apply(encoderConfig);

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
    SparkUtil.ifOk(
        motor,
        encoder::getVelocity,
        (value) -> inputs.accellerationDPSPS = (((value / 60.0) * 360.0) - inputs.wristDPS) / 0.02);

    SparkUtil.ifOk(motor, encoder::getPosition, (value) -> inputs.positionDegrees = value);

    SparkUtil.ifOk(motor, motor::getOutputCurrent, (value) -> inputs.motorAmps = value);
    SparkUtil.ifOk(
        motor, encoder::getVelocity, (value) -> inputs.wristDPS = (value / 60.0) * 360.0);

    pidController.setReference(setPointDegrees, ControlType.kPosition);
  }

  @Override
  public void setPositionDegrees(double positionDegrees) {
    this.setPointDegrees = positionDegrees;
  }

  @Override
  public void setBrakeMode(boolean enable) {
    var brakeConfig = new SparkMaxConfig();
    brakeConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }
}
