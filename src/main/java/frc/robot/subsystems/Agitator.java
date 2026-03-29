package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Agitator extends SubsystemBase {

    public static SparkMax agitatorLeft;
    public static SparkMax agitatorRight;

    private final RelativeEncoder agitatorLeftEncoder;
    private final RelativeEncoder agitatorRightEncoder;

    private static SparkClosedLoopController agitatorLeftPID;
    private static SparkClosedLoopController agitatorRightPID;

    private final SparkMaxConfig leftConfig  = new SparkMaxConfig();
    private final SparkMaxConfig rightConfig = new SparkMaxConfig();

    public static final double FORWARD_SOFT_LIMIT = 0;
    public static final double REVERSE_SOFT_LIMIT = -22;

    public Agitator() {
        agitatorLeft  = new SparkMax(Constants.SuperStructureConstants.kLeftClimber, MotorType.kBrushless);
        agitatorRight = new SparkMax(Constants.SuperStructureConstants.kRightClimber, MotorType.kBrushless);
        agitatorLeftEncoder  = agitatorLeft.getEncoder();
        agitatorRightEncoder = agitatorRight.getEncoder();

        agitatorLeftPID  = agitatorLeft.getClosedLoopController();
        agitatorRightPID = agitatorRight.getClosedLoopController();

//        SoftLimitConfig softLimits = new SoftLimitConfig()
//                .forwardSoftLimitEnabled(false)
//                .reverseSoftLimitEnabled(false);
//
//        leftConfig.apply(softLimits);
//        rightConfig.apply(softLimits);

        leftConfig
                .smartCurrentLimit(80)
                .inverted(false)
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .softLimit
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);
//                .inverted(false)
//                .smartCurrentLimit(Constants.ClimberConstants.kExtenderCurrentLimit)
//                .apply(pidConfig);

        leftConfig.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1); // 1 is for RPM
        leftConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.0013, 0, 0.016)
                .outputRange(-1, 1);

        rightConfig
                .smartCurrentLimit(80)
                .inverted(false)
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .softLimit
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);
//                .inverted(false)
//                .smartCurrentLimit(Constants.ClimberConstants.kExtenderCurrentLimit)
//                .apply(pidConfig);

        rightConfig.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1); // 1 is for RPM
        rightConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.0013, 0, 0.016)
                .outputRange(-1, 1);

        agitatorLeft.configure(leftConfig,   ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        agitatorRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static void stopMotors() {
        agitatorLeft.stopMotor();
        agitatorRight.stopMotor();
    }

    public static Command stopMotorsCommand() {
        return Commands.runOnce(() -> {
            agitatorLeft.stopMotor();
            agitatorRight.stopMotor();
        });
    }

    public static Command runReject() {
        return Commands.runEnd(
                () -> {
                    agitatorRight.setVoltage(4.5);
                    agitatorLeft.setVoltage(-4.5);
                },
                () -> {
                    agitatorRight.setVoltage(0);
                    agitatorLeft.setVoltage(0);
                }
        );
    }

    public static Command runRight() {
        return Commands.runEnd(
                () -> {
                    agitatorRight.setVoltage(4.5);
//                    agitatorRightPID.setSetpoint(1400, SparkBase.ControlType.kVelocity);
                },
                () -> {
//                    agitatorRightPID.setSetpoint(0, SparkBase.ControlType.kVelocity);
                    agitatorRight.setVoltage(0);
                }
        );
    }

    public static Command enableRight() {
        return Commands.runOnce(
                () -> {
                    agitatorRight.setVoltage(4.5);
//                    agitatorRightPID.setSetpoint(1400, SparkBase.ControlType.kVelocity);
                }
            );
    }

    public static Command runLeft() {
        return Commands.runEnd(
                () -> {
                    agitatorLeft.setVoltage(4.5);
//                    agitatorLeftPID.setSetpoint(1400, SparkBase.ControlType.kVelocity);
                },
                () -> {
//                    agitatorLeftPID.setSetpoint(0, SparkBase.ControlType.kVelocity);
                    agitatorLeft.setVoltage(0);
                }
        );
    }

    public static Command enableLeft() {
        return Commands.runOnce(
                () -> {
                    agitatorLeft.setVoltage(4.5);
//                    agitatorLeftPID.setSetpoint(1400, SparkBase.ControlType.kVelocity);
                }
            );
    }

    public double getLeftPosition() {
        return agitatorLeftEncoder.getPosition();
    }

    public double getRightPosition() {
        return agitatorRightEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Extender Left Position",  getLeftPosition());
        SmartDashboard.putNumber("Extender Right Position", getRightPosition());
    }

}