package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class SuperStructure extends SubsystemBase {
    public final SparkMax intake = new SparkMax(SuperStructureConstants.kIntake1, MotorType.kBrushless);
    private final SparkMax loader = new SparkMax(SuperStructureConstants.kLoader, MotorType.kBrushless);
    private final SparkFlex flywheel = new SparkFlex(SuperStructureConstants.kFlywheel, MotorType.kBrushless);
    private final SparkMax intake2 = new SparkMax(SuperStructureConstants.kIntake2, MotorType.kBrushless);
    private final SparkMax rotator = new SparkMax(SuperStructureConstants.kRotator, MotorType.kBrushless);
    private final SparkMax hood = new SparkMax(SuperStructureConstants.kHood, MotorType.kBrushless);

    private final SparkFlexConfig flywheelConfig = new SparkFlexConfig(); // Vortex
    private final SparkMaxConfig intakeConfig = new SparkMaxConfig(); // Neo
    private final SparkMaxConfig intake2Config = new SparkMaxConfig();
    private final SparkMaxConfig rotatorConfig = new SparkMaxConfig();
    private final SparkMaxConfig hoodConfig = new SparkMaxConfig();

    private final SparkClosedLoopController flywheelPID;
    private final SparkClosedLoopController rotatorPID;
    private final SparkClosedLoopController hoodPID;

    private double rotatorSetpoint = 0;
    private double hoodSetpoint = SuperStructureConstants.hoodStart;

    public SuperStructure() {
        flywheelPID = flywheel.getClosedLoopController();
        rotatorPID = rotator.getClosedLoopController();
        hoodPID = hood.getClosedLoopController();

        flywheelConfig
                .smartCurrentLimit(80)
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .softLimit
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);

        flywheelConfig.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1); // 1 is for RPM
        flywheelConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // .pid(0.0013, 0, 0.016)
                .pid(0.002, 0, 0.016)
                // .pid(0.002, 0, 0.0)
                // .pid(0.0, 0, 0.0)
                .outputRange(0, 1);

        intakeConfig
                .smartCurrentLimit(40)
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .softLimit
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);

        intake2Config
                .smartCurrentLimit(40)
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .softLimit
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);

        rotatorConfig
                .smartCurrentLimit(40)
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .softLimit
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);

        rotatorConfig.encoder
                .positionConversionFactor(360.0 * 1 / 9 * 1 / 9)
                .velocityConversionFactor(360.0 * 1 / 9 * 1 / 9);
        rotatorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.005, 0, 0)
                .outputRange(-1, 1);

        hoodConfig
                .smartCurrentLimit(30)
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .softLimit
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);

        hoodConfig.encoder
//            .positionConversionFactor(360.0 * 1/5 * 1/5 * 1/5 * 2/3);
//            .velocityConversionFactor(360.0 * 1/5 * 1/5 * 1/5 * 2/3);
                .positionConversionFactor(360.0 * 1 / 9 * 1 / 9 * 18 / 39)
                .velocityConversionFactor(360.0 * 1 / 9 * 1 / 9 * 18 / 39);
        hoodConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.1, 0, 0.05)
                .outputRange(-1, 1);

        flywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intake2.configure(intake2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rotator.configure(rotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hood.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // public static final double flywheelRPM = 3300; // 60 inches
    // public static final double flywheelRPM = 3500; // 70 inches
    // public static final double flywheelRPM = 3700; // 80 inches
    // public static final double flywheelRPM = 3900; // 90 inches
    // public static final double flywheelRPM = 4100; // 100 inches
    // public static final double flywheelRPM = 4300; // 110 inches
    // public static final double flywheelRPM = 4500; // 120 inches
    }

    //Flywheel Commands
    // shooter gun
    public Command primeFlywheel(double desiredRPM) {
        return Commands.runEnd(
                () -> {
//                flywheelConfig.closedLoop.pid(SmartDashboard.getNumber("pidp", 0), 0, SmartDashboard.getNumber("pidd", 0));
//                flywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//                    SmartDashboard.putBoolean("does work", true);
                    if(!Robot.useAutoHood) {
                        ShooterConstants.flywheelRPM = 4500;
                        setFlywheelRPM(ShooterConstants.flywheelRPM);
                    } else if (Robot.distanceToHub < 130.0) { // less than 130 inches then do this
                        setFlywheelRPM_FF_Short(Constants.ShooterConstants.flywheelRPM);
                    } else { // greater than 130 then do this
                        ShooterConstants.flywheelRPM = 4500;
                        setFlywheelRPM_FF_Far(desiredRPM);
                    }
                },
                () -> setFlywheelRPM(0)
        );
    }

    public Command enableFlywheel(double desiredRPM) {
        return Commands.runOnce(() -> {
            setFlywheelRPM(desiredRPM);
        });
    }

    public Command disableFlywheel() {
        return Commands.runOnce(() -> {
            setFlywheelRPM(0);
        });
    }

    //Intake Control Commands
    public Command runIntake() {
        return Commands.runEnd(
                () -> setIntakeVoltage(12),
                () -> setIntakeVoltage(0)
        );
    }

    public Command enableIntake() {
        return Commands.runOnce(
                () -> setIntakeVoltage(12)
        );
    }

    public Command disableIntake() {
        return Commands.runOnce(
                () -> setIntakeVoltage(0)
        );
    }

    public Command rejectIntake() {
        return Commands.runEnd(
                () -> setIntakeVoltage(-12),
                () -> setIntakeVoltage(0)
        );
    }

    public Command runIntake2() {
        return Commands.runEnd(
                () -> setIntake2Voltage(12),
                () -> setIntake2Voltage(0)
        );
    }
    public Command enableIntake2() {
        return Commands.runOnce(
                () -> setIntake2Voltage(12)
        );
    }

    public Command rejectIntake2() {
        return Commands.runEnd(
                () -> setIntake2Voltage(-12),
                () -> setIntake2Voltage(0)
        );
    }
    public Command disableIntake2() {
        return Commands.run(
                () -> setIntake2Voltage(0)
        );
    }

    public Command enableIntakes() {
        return Commands.runOnce(() -> {
            setIntakeVoltage(12);
            setIntake2Voltage(12);
            setLoaderVoltage(12);
        });
    }

    public Command rejectIntakes() {
        return Commands.runOnce(() -> {
            setIntakeVoltage(-12);
            setIntake2Voltage(-12);
            setLoaderVoltage(-12);
        });
    }

    public Command disableIntakes() {
        return Commands.runOnce(() -> {
            setIntakeVoltage(0);
            setIntake2Voltage(0);
            setLoaderVoltage(0);
        });
    }

    public Command longDistance() {
        return Commands.runEnd(
                () -> {
                    setHoodPos(SuperStructureConstants.hoodLongDistance);
                    // setFlywheelRPM(SuperStructureConstants.rpmLongDistance);
                },
                () -> setFlywheelRPM(0)
        );
    }

    public Command midDistance() {
        return Commands.runEnd(
                () -> {
                    setHoodPos(SuperStructureConstants.hoodMidDistance);
                    // setFlywheelRPM(SuperStructureConstants.rpmMidDistance);
                },
                () -> setFlywheelRPM(0)
        );
    }

    public Command shortDistance() {
        return Commands.runEnd(
                () -> {
                    setHoodPos(SuperStructureConstants.hoodShortDistance);
                    // setFlywheelRPM(SuperStructureConstants.rpmShortDistance);
                },
                () -> setFlywheelRPM(0)
        );
    }

    //Rotator Control Commands
    public Command runRotator(RobotContainer container) {
        return Commands.runOnce(
                () -> {
                    double fineControl = container.controller1.getRightY();
                    double deadBandControl = MathUtil.applyDeadband(fineControl, IOConstants.kDriveDeadband);
                    rotatorSetpoint += (deadBandControl * 0.1);
                    System.out.println("DBC: " + deadBandControl + " ROT: " + rotatorSetpoint + " FC: " + fineControl);
                    setRotatorPos(rotatorSetpoint);
                }
        );
    }

    public Command bringUpRotator() {
        return Commands.runEnd(
                () -> setRotatorPos(SuperStructureConstants.rotatorMin),
                () -> setRotatorPos(SuperStructureConstants.rotatorMax)
        );
    }

    public Command setRotator(double pos) {
        return Commands.runOnce(() -> setRotatorPos(pos));
    }

    //Loader Control Commands
    public Command runLoader() {
        return Commands.runEnd(
                () -> setLoaderVoltage(12),
                () -> setLoaderVoltage(0)
        );
    }

    public Command rejectLoader() {
        return Commands.runEnd(
                () -> setLoaderVoltage(-12),
                () -> setLoaderVoltage(0)
        );
    }

    public Command enableLoader() {
        return Commands.runOnce(
                () -> setLoaderVoltage(12)
        );
    }

    public Command enableRejectLoader() {
        return Commands.runOnce(
                () -> setLoaderVoltage(-12)
        );
    }

    public Command disableLoader() {
        return Commands.runOnce(
                () -> setLoaderVoltage(0)
        );
    }

    public Command rejectLoaderAuto() {
        return Commands.runEnd(
                () -> {
                    Robot.loadingFuel = true;
                    setLoaderVoltage(-12);
                },
                () -> {
                    Robot.loadingFuel = false;
                    setLoaderVoltage(0);
                }
        );
    }

    public void setFlywheelRPM(double rpm) {
        rpm = fixRPM(rpm); // basically pid with simple approximate feedforward
        flywheelPID.setSetpoint(rpm, ControlType.kVelocity);
    }

    public void setFlywheelRPM_FF_Far(double rpm) {
        // rpm = fixRPM(rpm); // basically pid with simple approximate feedforward
        flywheelPID.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, 8.5);
    }

    public void setFlywheelRPM_FF_Short(double rpm) {
        // rpm = fixRPM(rpm); // basically pid with simple approximate feedforward
        flywheelPID.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, 6);
    }

    public void setIntakeVoltage(double voltage) {
        SmartDashboard.putNumber("Intake Setting", voltage);
        intake.setVoltage(voltage);
    }

    public void setIntake2Voltage(double voltage) {
        SmartDashboard.putNumber("Intake2 Setting", voltage);
        intake2.setVoltage(voltage);
    }

    public void setRotatorPos(double position) {
        rotatorSetpoint = filterValue(position, SuperStructureConstants.rotatorMin, SuperStructureConstants.rotatorMax);
        SmartDashboard.putNumber("Rotator Target", rotatorSetpoint);
        rotatorPID.setSetpoint(rotatorSetpoint, ControlType.kPosition);
    }

    public void setLoaderVoltage(double voltage) {
        SmartDashboard.putNumber("Loader Setting", voltage);
        loader.setVoltage(voltage);
    }

    public void setHoodPos(double target) {
        hoodSetpoint = filterValue(target, SuperStructureConstants.hoodMin, SuperStructureConstants.hoodMax);
        hoodPID.setSetpoint(hoodSetpoint, ControlType.kPosition);
    }

    public double getRPM() {
        return flywheel.getEncoder().getVelocity();
    }

    public double getRotatorPos() {
        return rotator.getEncoder().getPosition();
    }

    public double getRotatorTarget() {
        return rotatorSetpoint;
    }

    public double getHoodPos() {
        return hood.getEncoder().getPosition();
    }

    public double getHoodTarget() {
        return hoodSetpoint;
    }

    /* Increases RPM by 20% */
    public double fixRPM(double rpm) {
        return rpm / 0.8;
    }

    /**
     * Returns the max if the setpoint is above it, or the hard deck if the setpoint is below it.
     * Otherwise, returns the setpoint.
     *
     * @param value The desired state of the crane.
     * @param min   The hard deck of the crane.
     */
    public double filterValue(double value, double min, double max) {
        if (value < min)
            value = min;
        if (value > max)
            value = max;
        return value;
    }
}