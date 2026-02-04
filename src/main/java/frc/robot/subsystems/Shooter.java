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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    SparkMax loader = new SparkMax(2, MotorType.kBrushless);
    SparkFlex flywheel = new SparkFlex(3, MotorType.kBrushless);
    SparkMax intake = new SparkMax(1, MotorType.kBrushless);
    
    SparkMaxConfig loaderConfig = new SparkMaxConfig(); // Neo
    SparkFlexConfig flywheelConfig = new SparkFlexConfig(); // Vortex
    SparkMaxConfig intakeConfig = new SparkMaxConfig(); // Neo

    public double flywheelSetpoint = 0; // target RPM

    SparkClosedLoopController flywheelPID;

    


    // boolean intakeState = false;

    public Shooter() {
        flywheelPID = flywheel.getClosedLoopController();


        loaderConfig
            .smartCurrentLimit(40)
            .inverted(true)
            .idleMode(IdleMode.kBrake)
        .softLimit
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false);
            
        flywheelConfig
            .smartCurrentLimit(80)
            .inverted(true)
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
            .p(0.0007)
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

        loader.configure(loaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    // public void toggleIntake(double voltage) {
    //     intakeState = !intakeState;
    //     if(intakeState == true) {
    //         setIntakeVoltage(voltage);
    //     } else {
    //         setIntakeVoltage(0);
    //     }
    // }

    // public void setIntakeState(boolean state) {
    //     intakeState = state;
    // }

    public void setFlywheelRPM(double rpm) {
        flywheelSetpoint = rpm;
        flywheelPID.setSetpoint(rpm, ControlType.kVelocity);
    }

    public double getRPM() {
        return flywheel.getEncoder().getVelocity();
    }

    public void primeFlywheel() {
        setFlywheelRPM(4000);
    }

    public void stopFlywheel() {
        setFlywheelRPM(0);
    }

    public void setIntakeVoltage(double voltage) {
        intake.setVoltage(voltage);
    }
    
    public void setFlywheelVoltage(double voltage) {
        flywheel.setVoltage(voltage);
    }
    
    public void setLoaderVoltage(double voltage) {
        loader.setVoltage(voltage);
    }
}
