package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    public static SparkMax extenderLeft;
    public static SparkMax extenderRight;

    private final RelativeEncoder extenderLeftEncoder;
    private final RelativeEncoder extenderRightEncoder;

    private final SparkClosedLoopController extenderLeftPID;
    private final SparkClosedLoopController extenderRightPID;

    private final SparkMaxConfig leftConfig  = new SparkMaxConfig();
    private final SparkMaxConfig rightConfig = new SparkMaxConfig();

    public static final double FORWARD_SOFT_LIMIT = 0;
    public static final double REVERSE_SOFT_LIMIT = -22;

    public Climber() {
        extenderLeft  = new SparkMax(Constants.SuperStructureConstants.kLeftClimber, MotorType.kBrushless);
        extenderRight = new SparkMax(Constants.SuperStructureConstants.kRightClimber, MotorType.kBrushless);

        ClosedLoopConfig pidConfig = new ClosedLoopConfig()
                .p(Constants.ClimberConstants.kExtenderKP)
                .outputRange(-1, 1);

        SoftLimitConfig softLimits = new SoftLimitConfig()
                .forwardSoftLimit(FORWARD_SOFT_LIMIT)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(REVERSE_SOFT_LIMIT)
                .reverseSoftLimitEnabled(true);

        leftConfig
                .inverted(true)
                .smartCurrentLimit(Constants.ClimberConstants.kExtenderCurrentLimit)
//                .apply(softLimits)
                .apply(pidConfig);

        rightConfig
                .inverted(true)
                .smartCurrentLimit(Constants.ClimberConstants.kExtenderCurrentLimit)
//                .apply(softLimits)
                .apply(pidConfig);

        extenderLeft.configure(leftConfig,   ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        extenderRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        extenderLeftEncoder  = extenderLeft.getEncoder();
        extenderRightEncoder = extenderRight.getEncoder();

        extenderLeftPID  = extenderLeft.getClosedLoopController();
        extenderRightPID = extenderRight.getClosedLoopController();
    }

    public void setSoftLimitsEnabled(boolean enabled) {
        SoftLimitConfig softLimits = new SoftLimitConfig()
                .forwardSoftLimitEnabled(enabled)
                .reverseSoftLimitEnabled(enabled);

        leftConfig.apply(softLimits);
        rightConfig.apply(softLimits);

        extenderLeft.configure(leftConfig,   ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        extenderRight.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setExtender(double setPoint) {
        extenderLeftPID.setSetpoint(setPoint,  ControlType.kPosition);
        extenderRightPID.setSetpoint(setPoint, ControlType.kPosition);
        SmartDashboard.putNumber("Extender Setpoint", setPoint);
    }

    public void setExtenderRaw(double speed) {
        extenderLeft.set(speed);
        extenderRight.set(speed);
    }

    public void stopExtender() {
        extenderLeft.stopMotor();
        extenderRight.stopMotor();
    }

    public double getLeftPosition() {
        return extenderLeftEncoder.getPosition();
    }

    public double getRightPosition() {
        return extenderRightEncoder.getPosition();
    }

    public void crawlUp() {
//        setExtenderRaw(12);
        extenderLeft.setVoltage(12);
        extenderRight.setVoltage(12);
    }

    public void crawlDown() {
//        setExtenderRaw(-12);
        extenderLeft.setVoltage(-12);
        extenderRight.setVoltage(-12);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Extender Left Position",  getLeftPosition());
        SmartDashboard.putNumber("Extender Right Position", getRightPosition());
    }

    public static Command crawlRight() {
        return Commands.runEnd(
                () -> {
                    extenderRight.setVoltage(12);
                },
                () -> {
                    extenderRight.setVoltage(0);
                }
        );
    }

}