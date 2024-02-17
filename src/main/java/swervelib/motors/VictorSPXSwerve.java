package swervelib.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.parser.PIDFConfig;
import swervelib.telemetry.Alert;

/**
 * Brushed motor control with SparkMax.
 */
public class VictorSPXSwerve extends SwerveMotor
{

  /**
   * VictorSPX Instance.
   */
  public VictorSPX motor;

  /**
   * Absolute encoder attached to the SparkMax (if exists)
   */
  public SwerveAbsoluteEncoder absoluteEncoder;

  /**
   * Closed-loop PID controller.
   */
  public PIDController pid;
  public double pid_iz;
  public double pid_f;
  public double pid_output_min;
  public double pid_output_max;

  /**
   *
   */
  public SimpleMotorFeedforward feedforward;

  /**
   * Current VictorSPX configuration.
   */
  private final VictorSPXConfiguration configuration = new VictorSPXConfiguration();

  /**
   * The position conversion factor to convert raw sensor units to Meters Per 100ms, or Ticks to Degrees.
   */
  private double  positionConversionFactor = 360;

  /**
   * If the VictorSPX configuration has changed.
   */
  private boolean configChanged            = true;

  /**
   * Nominal voltage default to use with feedforward.
   */
  private double  nominalVoltage           = 12.0;

  /**
   * Factory default already occurred.
   */
  private boolean            factoryDefaultOccurred = false;

  /**
   * An {@link Alert} for if the motor has no encoder defined.
   */
  private Alert              noEncoderDefinedAlert;

  /**
   * Initialize the swerve motor.
   *
   * @param motor                  The SwerveMotor as a SparkMax object.
   */
  public VictorSPXSwerve(VictorSPX motor)
  {
    this.isDriveMotor = false;
    this.motor = motor;

    // noEncoderDefinedAlert = new Alert("Motors",
    //                                   "An encoder MUST be defined to work with a VictorSPX",
    //                                   Alert.AlertType.ERROR_TRACE);

    motor.configSelectedFeedbackSensor(FeedbackDevice.None);

    factoryDefaults();
    clearStickyFaults();

    // Create an empty pid controller
    pid = new PIDController(0,0,0);
  }

  /**
   * Initialize the {@link SwerveMotor} as a {@link CANSparkMax} connected to a Brushless Motor.
   *
   * @param id                     CAN ID of the SparkMax.
   * @param isDriveMotor           Is the motor being initialized a drive motor?
   * @param encoderType            {@link Type} of encoder to use for the {@link CANSparkMax} device.
   * @param countsPerRevolution    The number of encoder pulses for the {@link Type} encoder per revolution.
   * @param useDataPortQuadEncoder Use the encoder attached to the data port of the spark max for a quadrature encoder.
   */
  public VictorSPXSwerve(int id)
  {
    this(new VictorSPX(id));
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   */
  @Override
  public void setVoltageCompensation(double nominalVoltage)
  {
    configuration.voltageCompSaturation = nominalVoltage;
    configChanged = true;
  }

  /**
   * Set the current limit for the swerve drive motor, remember this may cause jumping if used in conjunction with
   * voltage compensation. This is useful to protect the motor from current spikes.
   *
   * @param currentLimit Current limit in AMPS at free speed.
   */
  @Override
  public void setCurrentLimit(int currentLimit)
  {
    // not supported
    // configuration.continuousCurrentLimit = currentLimit;
    // configuration.peakCurrentLimit = currentLimit;
    // configChanged = true;
  }

  /**
   * Set the maximum rate the open/closed loop output can change by.
   *
   * @param rampRate Time in seconds to go from 0 to full throttle.
   */
  @Override
  public void setLoopRampRate(double rampRate)
  {
    configuration.closedloopRamp = rampRate;
    configuration.openloopRamp = rampRate;
    configChanged = true;
  }

  /**
   * Get the motor object from the module.
   *
   * @return Motor object.
   */
  @Override
  public Object getMotor()
  {
    return motor;
  }

  /**
   * Queries whether the absolute encoder is directly attached to the motor controller.
   *
   * @return connected absolute encoder state.
   */
  @Override
  public boolean isAttachedAbsoluteEncoder()
  {
    return absoluteEncoder != null;
    // return false;
  }

  public void configureCANStatusFrames(int CANStatus1)
  {
    motor.setStatusFramePeriod(StatusFrame.Status_1_General, CANStatus1);
  }

  /**
   * Set the CAN status frames.
   *
   * @param CANStatus1       Applied Motor Output, Fault Information, Limit Switch Information
   * @param CANStatus2       Selected Sensor Position (PID 0), Selected Sensor Velocity (PID 0), Brushed Supply Current
   *                         Measurement, Sticky Fault Information
   * @param CANStatus3       Quadrature Information
   * @param CANStatus4       Analog Input, Supply Battery Voltage, Controller Temperature
   * @param CANStatus8       Pulse Width Information
   * @param CANStatus10      Motion Profiling/Motion Magic Information
   * @param CANStatus12      Selected Sensor Position (Aux PID 1), Selected Sensor Velocity (Aux PID 1)
   * @param CANStatus13      PID0 (Primary PID) Information
   * @param CANStatus14      PID1 (Auxiliary PID) Information
   * @param CANStatus21      Integrated Sensor Position (Talon FX), Integrated Sensor Velocity (Talon FX)
   * @param CANStatusCurrent Brushless Supply Current Measurement, Brushless Stator Current Measurement
   */
  public void configureCANStatusFrames(int CANStatus1, int CANStatus2, int CANStatus4,
                                       int CANStatus10, int CANStatus12, int CANStatus13, int CANStatus14)
  {
    motor.setStatusFramePeriod(StatusFrame.Status_1_General, CANStatus1);
    motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, CANStatus2);
    motor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, CANStatus4);
    motor.setStatusFramePeriod(StatusFrame.Status_10_Targets, CANStatus10);
    motor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, CANStatus12);
    motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, CANStatus13);
    motor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, CANStatus14);

    // TODO: Configure Status Frame 2 thru 14 if necessary
    // https://v5.docs.ctr-electronics.com/en/stable/ch18_CommonAPI.html#setting-status-frame-periods
  }

  /**
   * Configure the factory defaults.
   */
  @Override
  public void factoryDefaults()
  {
    if (!factoryDefaultOccurred)
    {
      motor.configFactoryDefault();
      motor.setSensorPhase(true);
    }
  }

  /**
   * Clear the sticky faults on the motor controller.
   */
  @Override
  public void clearStickyFaults()
  {
    motor.clearStickyFaults();
  }

  /**
   * Set the absolute encoder to be a compatible absolute encoder.
   *
   * @param encoder The encoder to use.
   * @return The {@link SwerveMotor} for easy instantiation.
   */
  @Override
  public SwerveMotor setAbsoluteEncoder(SwerveAbsoluteEncoder encoder)
  {
    absoluteEncoder = encoder;

    if (absoluteEncoder == null)
    {
      noEncoderDefinedAlert.set(true);
      throw new RuntimeException("An encoder MUST be defined to work with a VictorSPX");
    }
    return this;
  }

  /**
   * Configure the integrated encoder for the swerve module. Sets the conversion factors for position and velocity.
   *
   * @param positionConversionFactor The conversion factor to apply.
   */
  @Override
  public void configureIntegratedEncoder(double positionConversionFactor)
  {
    // Encoder yoksa ağla
    if (absoluteEncoder == null)
    {
      noEncoderDefinedAlert.set(true);
      throw new RuntimeException("An encoder MUST be defined to work with a VictorSPX");
    }

    this.positionConversionFactor = positionConversionFactor;

    // burada sparkmaxbrushedtan kalan yeri bıraktım ama
    // cancoder için ne kadar iyi çalışacak bilmiyorum
    // absoluteEncoder.setPositionConversionFactor(positionConversionFactor);
    // absoluteEncoder.setVelocityConversionFactor(positionConversionFactor / 60);

    // Taken from democat's library.
    // https://github.com/democat3457/swerve-lib/blob/7c03126b8c22f23a501b2c2742f9d173a5bcbc40/src/main/java/com/swervedrivespecialties/swervelib/ctre/Falcon500DriveControllerFactoryBuilder.java#L16
    configureCANStatusFrames(250);
  }

  /**
   * Configure the PIDF values for the closed loop controller.
   *
   * @param config Configuration class holding the PIDF values.
   */
  @Override
  public void configurePIDF(PIDFConfig config)
  {
    pid.setPID(config.p, config.i, config.d);
    pid_f = config.f;
    pid_iz = config.iz;
    pid_output_min = config.output.min;
    pid_output_max = config.output.max;
  }

  /**
   * Configure the PID wrapping for the position closed loop controller.
   *
   * @param minInput Minimum PID input.
   * @param maxInput Maximum PID input.
   */
  @Override
  public void configurePIDWrapping(double minInput, double maxInput)
  {
    // not sure if i should implement this

    // configureSparkMax(() -> pid.setPositionPIDWrappingEnabled(true));
    // configureSparkMax(() -> pid.setPositionPIDWrappingMinInput(minInput));
    // configureSparkMax(() -> pid.setPositionPIDWrappingMaxInput(maxInput));
  }

  /**
   * Set the CAN status frames.
   *
   * @param CANStatus0 Applied Output, Faults, Sticky Faults, Is Follower
   * @param CANStatus1 Motor Velocity, Motor Temperature, Motor Voltage, Motor Current
   * @param CANStatus2 Motor Position
   * @param CANStatus3 Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
   * @param CANStatus4 Alternate Encoder Velocity, Alternate Encoder Position
   */
  /**
   * Set the idle mode.
   *
   * @param isBrakeMode Set the brake mode.
   */
  @Override
  public void setMotorBrake(boolean isBrakeMode)
  {
    motor.setNeutralMode(isBrakeMode ? NeutralMode.Brake : NeutralMode.Coast);
  }

  /**
   * Set the motor to be inverted.
   *
   * @param inverted State of inversion.
   */
  @Override
  public void setInverted(boolean inverted)
  {
    motor.setInverted(inverted);
  }

  /**
   * Save the configurations from flash to EEPROM.
   */
  @Override
  public void burnFlash()
  {
    if (configChanged)
    {
      motor.configAllSettings(configuration, 250);
      configChanged = false;
    }
  }

  /**
   * Set the percentage output.
   *
   * @param percentOutput percent out for the motor controller.
   */
  @Override
  public void set(double percentOutput)
  {
    motor.set(VictorSPXControlMode.PercentOutput, percentOutput);
  }

  /**
   * Set the closed loop PID controller reference point.
   *
   * @param setpoint    Setpoint in MPS or Angle in degrees.
   * @param feedforward Feedforward in volt-meter-per-second or kV.
   */
  @Override
  public void setReference(double setpoint, double feedforward)
  {
    // throw new RuntimeException("Cannot use setReference without position for a VictorSPX, use setReference(double, double, double) instead.");
    setReference(setpoint, feedforward, getPosition());
  }

  /**
   * Set the closed loop PID controller reference point.
   *
   * @param setpoint    Setpoint in meters per second or angle in degrees.
   * @param feedforward Feedforward in volt-meter-per-second or kV.
   * @param position    Only used on the angle motor, the position of the motor in degrees.
   */
  @Override
  public void setReference(double setpoint, double feedforward, double position)
  {
    burnFlash();

    // Apply the integral zone limit
    if (Math.abs(pid.getPositionError()) > pid_iz) {
        pid.reset();
    }

    double realSetpoint = convertToNativeSensorUnits(setpoint, position);
    double pidOutput = pid.calculate(position, realSetpoint);

    // Calculate normal feedforward based on setpoint and nominal voltage
    // Normal feedforward (f) from PIDFConfig is used here
    double normalFeedforward = (pid_f * setpoint) / nominalVoltage;

    // Adjust arbitrary feedforward by nominal voltage for consistency
    double adjustedArbitraryFeedforward = feedforward / nominalVoltage;

    // Combine PID output with both feedforward values, ensuring the result is within [-1, 1]
    double totalOutput = pidOutput + normalFeedforward + adjustedArbitraryFeedforward;
    double cutOutput = Math.max(pid_output_min, Math.min(pid_output_max, totalOutput));

    motor.set(ControlMode.PercentOutput, cutOutput);
  }

  /**
   * Get the voltage output of the motor controller.
   *
   * @return Voltage output.
   */
  @Override
  public double getVoltage()
  {
    return motor.getMotorOutputVoltage();
  }

  /**
   * Set the voltage of the motor.
   *
   * @param voltage Voltage to set.
   */
  @Override
  public void setVoltage(double voltage)
  {
    // Not supported
  }

  /**
   * Get the applied dutycycle output.
   *
   * @return Applied dutycycle output to the motor.
   */
  @Override
  public double getAppliedOutput()
  {
    return motor.getMotorOutputPercent();
  }

  /**
   * Get the velocity of the integrated encoder.
   *
   * @return velocity
   */
  @Override
  public double getVelocity()
  {
    // throw new RuntimeException("Cannot get velocity from a VictorSPX, use an absolute encoder instead.");
    return absoluteEncoder.getVelocity();
  }

  /**
   * Get the position of the integrated encoder.
   *
   * @return Position
   */
  @Override
  public double getPosition()
  {
    return absoluteEncoder.getAbsolutePosition();
    // throw new RuntimeException("Cannot get position from a VictorSPX, use an absolute encoder instead.");
  }

  /**
   * Set the integrated encoder position.
   *
   * @param position Integrated encoder position.
   * THERE IS NO INTEGRATED ENCODER
   * this function is normally used to synchronize the
   * absolute encoders and the integrated ones in the angle motors
   * but they are the same so do nothing
   */
  @Override
  public void setPosition(double position)
  {
  }

  public double convertToNativeSensorUnits(double setpoint, double position)
  {
    setpoint = isDriveMotor ? setpoint * .1 : placeInAppropriate0To360Scope(position, setpoint);
    return setpoint / positionConversionFactor;
  }

  /**
   * Put an angle within the 360 deg scope of a reference. For example, given a scope reference of 756 degrees, assumes
   * the full scope is (720-1080), and places an angle of 22 degrees into it, returning 742 deg.
   *
   * @param scopeReference Current Angle (deg)
   * @param newAngle       Target Angle (deg)
   * @return Closest angle within scope (deg)
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle)
  {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0)
    {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else
    {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound)
    {
      newAngle += 360;
    }
    while (newAngle > upperBound)
    {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180)
    {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180)
    {
      newAngle += 360;
    }
    return newAngle;
  }

}
