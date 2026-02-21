package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.RobotContainer;

public class SuperStructure extends SubsystemBase {
    private final SparkMax loader = new SparkMax(2, MotorType.kBrushless);
    private final SparkFlex flywheel = new SparkFlex(3, MotorType.kBrushless);
    private final SparkMax intake = new SparkMax(1, MotorType.kBrushless);
    private final SparkMax intake2 = new SparkMax(SuperStructureConstants.kIntake2, MotorType.kBrushless);
    private final SparkMax rotator = new SparkMax(SuperStructureConstants.kRotator, MotorType.kBrushless);

    private final SparkMaxConfig loaderConfig = new SparkMaxConfig(); // Neo
    private final SparkFlexConfig flywheelConfig = new SparkFlexConfig(); // Vortex
    private final SparkMaxConfig intakeConfig = new SparkMaxConfig(); // Neo
    private final SparkMaxConfig intake2Config = new SparkMaxConfig();
    private final SparkMaxConfig rotatorConfig = new SparkMaxConfig();

    private final SparkClosedLoopController flywheelPID;
    private final SparkClosedLoopController rotatorPID;

    private double rotatorSetpoint = 0;

    public SuperStructure() {
        flywheelPID = flywheel.getClosedLoopController();
        rotatorPID = rotator.getClosedLoopController();

        loaderConfig
            .smartCurrentLimit(40)
            .inverted(true)
            .idleMode(IdleMode.kBrake)
        .softLimit
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false);
            
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
            // .pid(0.005,0,0)
            .p(0.002)
            .i(0.0)
            .d(0.0)
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
            .positionConversionFactor(360.0 * 1/9 * 1/9)
            .velocityConversionFactor(360.0 * 1/9 * 1/9);
        rotatorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // .pid(0.005,0,0)
            .p(0.005)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1);

        loader.configure(loaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intake2.configure(intake2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rotator.configure(rotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    //Flywheel Commands
    public Command primeFlywheel(double desiredRPM) {
        return Commands.runEnd(
            () -> {
//                flywheelConfig.closedLoop.pid(SmartDashboard.getNumber("pidp", 0), 0, SmartDashboard.getNumber("pidd", 0));
//                flywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                setFlywheelRPM(desiredRPM);
            },
            () -> setFlywheelRPM(0)
        );
    }

    //Intake Control Commands
    public Command runIntake() {
        return Commands.runEnd(
            () -> setIntakeVoltage(12), 
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

    public Command rejectIntake2() {
        return Commands.runEnd(
            () -> setIntake2Voltage(-12),
            () -> setIntake2Voltage(0)
        );
    }

    //Rotator Control Commands
    public Command runRotator(RobotContainer container) {
        return Commands.runOnce(
            () -> {
                double fineControl = container.controller1.getRightY();
                double deadBandControl = MathUtil.applyDeadband(fineControl, IOConstants.kDriveDeadband);
                rotatorSetpoint += (deadBandControl*0.1);
                System.out.println("DBC: " + deadBandControl + " ROT: " + rotatorSetpoint + " FC: " + fineControl);
                setRotatorPos(rotatorSetpoint);
            }
        );
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

    //Control methods for all superstructure subsystems
    //Use these in the commands above to apply setpoints and voltages
    private void setFlywheelRPM(double rpm) {
        SmartDashboard.putNumber("Flywheel Setting", rpm);
        rpm = fixRPM(rpm); // basically pid with simple approximate feedforward
        flywheelPID.setSetpoint(rpm, ControlType.kVelocity);
    }

    private void setIntakeVoltage(double voltage) {
        SmartDashboard.putNumber("Intake Setting", voltage);
        intake.setVoltage(voltage);
    }

    private void setIntake2Voltage(double voltage) {
        SmartDashboard.putNumber("Intake2 Setting", voltage);
        intake2.setVoltage(voltage);
    }

    private void setRotatorPos(double position){
        rotatorSetpoint = filterValue(position, SuperStructureConstants.rotatorMin, SuperStructureConstants.rotatorMax);
        SmartDashboard.putNumber("Rotator Target", rotatorSetpoint);
        rotatorPID.setSetpoint(rotatorSetpoint, ControlType.kPosition);
    }
    
    private void setLoaderVoltage(double voltage) {
        SmartDashboard.putNumber("Loader Setting", voltage);
        loader.setVoltage(voltage);
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

    /* Increases RPM by 20% */
    public double fixRPM(double rpm) {
        return rpm / 0.8;
    }


    /**
     * Returns the max if the setpoint is above it, or the hard deck if the setpoint is below it.
     * Otherwise, returns the setpoint.
     * 
     * @param value The desired state of the crane.
     * @param min The hard deck of the crane.
     */
    private double filterValue(double value, double min, double max) {
        if(value < min)
            value = min;
        if(value > max)
            value = max;
        return value;
    }
}