package team5427.frc.robot.subsystems.Swerve.io;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Queue;
import java.util.function.DoubleSupplier;
import team5427.frc.robot.Constants;
import team5427.frc.robot.subsystems.Swerve.SwerveConstants;
import team5427.frc.robot.subsystems.Swerve.io.ModuleIO;
import team5427.frc.robot.subsystems.Swerve.io.spark.SparkOdometryThread;
import team5427.frc.robot.subsystems.Swerve.io.talon.PhoenixOdometryThread;
import team5427.lib.motors.SimpleSparkMax;
import team5427.lib.motors.SparkUtil;
import team5427.lib.motors.SteelTalonFX;

public class ModuleIOHybrid implements ModuleIO {
  private SimpleSparkMax steerMotor;
  private SteelTalonFX driveMotor;

  private CANcoder cancoder;

  private final int moduleIdx;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private StatusSignal<Angle> absolutePosition;

  private SwerveModuleState targetModuleState;
  private StatusSignal<Voltage> driveMotorVoltage;
  private StatusSignal<Current> driveMotorCurrent;
  private StatusSignal<Current> driveTorqueCurrent;
  private StatusSignal<Angle> driveMotorPosition;
  private StatusSignal<AngularVelocity> driveMotorVelocity;

  public ModuleIOHybrid(int index) {
    this.moduleIdx = index;
    steerMotor = new SimpleSparkMax(SwerveConstants.kSwerveUtilInstance.kSteerMotorIds[index]);
    driveMotor = new SteelTalonFX(SwerveConstants.kSwerveUtilInstance.kDriveMotorIds[index]);
    cancoder =
        new CANcoder(SwerveConstants.kSwerveUtilInstance.kCancoderIds[index].getDeviceNumber());

    steerMotor.apply(SwerveConstants.kSteerMotorConfiguration);
    driveMotor.apply(SwerveConstants.kDriveMotorConfiguration);

    driveMotor.apply(SwerveConstants.kDriveMotorConfiguration);

    steerMotor
        .getSparkMaxConfig()
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / Constants.kOdometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    steerMotor.applySparkConfiguration(steerMotor.getSparkMaxConfig());

    CANcoderConfiguration configuration = new CANcoderConfiguration();
    configuration.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
    configuration.MagnetSensor.MagnetOffset =
        SwerveConstants.kSwerveUtilInstance.kModuleOffsets[moduleIdx];
    configuration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoder.getConfigurator().apply(configuration);

    cancoder.clearStickyFaults();

    absolutePosition = cancoder.getAbsolutePosition();

    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    this.drivePositionQueue =
    PhoenixOdometryThread.getInstance().registerSignal(this.driveMotorPosition);
    turnPositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(
                steerMotor.getSparkMax(), steerMotor.getSparkMax().getEncoder()::getPosition);
    driveMotorPosition = driveMotor.getTalonFX().getPosition();

    driveMotorVelocity = driveMotor.getTalonFX().getVelocity();

    System.out.println("New Module with idx: " + moduleIdx);

    driveMotorVoltage = driveMotor.getTalonFX().getMotorVoltage();
    driveMotorCurrent = driveMotor.getTalonFX().getStatorCurrent();
    driveTorqueCurrent = driveMotor.getTalonFX().getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.kOdometryFrequency, driveMotorPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveMotorVoltage,
        driveMotorCurrent,
        absolutePosition,
        driveMotorVelocity);

    ParentDevice.optimizeBusUtilizationForAll(
        driveMotor.getTalonFX(), cancoder);

    BaseStatusSignal.waitForAll(
        0.02, absolutePosition, driveMotorPosition, driveMotorVelocity);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
     BaseStatusSignal.refreshAll(driveMotorPosition);

    BaseStatusSignal.refreshAll(
        driveMotorVoltage,
        driveMotorCurrent,
        driveMotorVelocity,
        absolutePosition);
    inputs.absolutePosition = Rotation2d.fromRotations(absolutePosition.getValue().in(Rotations));
    inputs.driveMotorPosition = Rotation2d.fromRadians(driveMotorPosition.getValue().in(Radian));
    inputs.driveMotorAngularVelocity = driveMotorVelocity.getValue();
    inputs.driveMotorLinearVelocity =
        MetersPerSecond.of(
            driveMotorVelocity.getValue().in(RotationsPerSecond)
                * Math.PI
                * driveMotor.getMotorConfiguration().finalDiameterMeters);
    SparkUtil.ifOk(
        steerMotor.getSparkMax(),
        steerMotor::getEncoderPosition,
        (value) -> inputs.steerPosition = Rotation2d.fromRotations(value));
    inputs.currentModuleState =
        new SwerveModuleState(inputs.driveMotorLinearVelocity, inputs.absolutePosition);
    inputs.currentModulePosition = new SwerveModulePosition(inputs.driveMotorLinearVelocity.in(MetersPerSecond), inputs.steerPosition);
    SparkUtil.ifOk(
        steerMotor.getSparkMax(),
        steerMotor.getSparkMax()::getOutputCurrent,
        (value) -> inputs.steerMotorCurrent = Amp.of(value));

    SparkUtil.ifOk(
        steerMotor.getSparkMax(),
        new DoubleSupplier[] {
          steerMotor.getSparkMax()::getAppliedOutput, steerMotor.getSparkMax()::getBusVoltage
        },
        (values) -> inputs.steerMotorVoltage = Volts.of(values[0] * values[1]));

    inputs.driveMotorConnected = driveMotor.getTalonFX().isConnected();
    inputs.steerMotorConnected = !steerMotor.getSparkMax().hasStickyFault();

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsMeters =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> value * driveMotor.getConversionFactorFromRotations())
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  public void setDriveSpeedSetpoint(LinearVelocity speed) {
    driveMotor.setSetpoint(speed);
  }

  /*
   * Needs a voltage
   */
  public void setDriveSpeedSetpoint(Voltage volts) {
    driveMotor.setRawVoltage(volts);
  }

  public void setDriveSpeedSetpoint(Current current) {
    driveMotor.setRawCurrent(current);
  }

  public void setDriveFeedForward(Current current) {
    driveMotor.getMotorConfiguration().kFF = current.in(Amps);
  }

  public void setDriveFeedForward(Force xForce, Force yForce) {
    DriverStation.reportWarning(
        "Talon FX Drive Motor with id: "
            + driveMotor.getTalonFX().getDeviceID()
            + " is assigned a force feedforward. This library currently does not support force feedforwards due to unfinished implementation.",
        false);
  }

  public void setSteerPositionSetpoint(Rotation2d position) {
    steerMotor.setSetpoint(position);
  }

  /*
   * Needs a voltage
   */
  public void setSteerPositionSetpoint(Voltage volts) {
    steerMotor.setRawVoltage(volts);
  }

  public void setSteerPositionSetpoint(Current current) {
    steerMotor.setRawCurrent(current);
  }
  ;

  public void setModuleState(SwerveModuleState state) {
    setDriveSpeedSetpoint(MetersPerSecond.of(state.speedMetersPerSecond));
    setSteerPositionSetpoint(state.angle);
  }

  public void resetMotorSetpoint(Rotation2d steerPosition) {
    driveMotor.setEncoderPosition(0.0);
    steerMotor.setEncoderPosition(steerPosition);
  }
}
