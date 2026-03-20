package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class climberOLD extends SubsystemBase {
    public climberOLD() {

    }

//    public boolean onClimberControl = false;
//
//    public double climbLeftSetPoint = 0.0;
//    public double climbRightSetPoint = 0.0;
//    private final SparkMaxConfig climbConfig = new SparkMaxConfig();
//
//    private final SparkMax climbLeft;
//    private final SparkMax climbRight;
//
//    private final SparkClosedLoopController climbLeftPID;
//    private final SparkClosedLoopController climbRightPID;
//
//    public climberOLD() {
//        climbLeft = new SparkMax(Constants.SuperStructureConstants.kLeftClimber, SparkLowLevel.MotorType.kBrushless);
//        climbRight = new SparkMax(Constants.SuperStructureConstants.kRightClimber, SparkLowLevel.MotorType.kBrushless);
//
//        climbConfig
//                .smartCurrentLimit(100)
//                .idleMode(IdleMode.kBrake);
//        climbConfig.softLimit
//                .forwardSoftLimitEnabled(true)
//                .reverseSoftLimitEnabled(true);
////                .forwardSoftLimit(ClimberConstants.kAnchorCeiling)
////                .reverseSoftLimit(ClimberConstants.kAnchorHardDeck); // NEED TO IMPLEMENT
//        climbConfig.encoder
//                .positionConversionFactor(Constants.ClimberConstants.kAnchorEncoderDistancePerPulse);
//        climbConfig.closedLoop
//                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//                .pid(0.05, 0.0, 0.1);
//
//        climbLeft.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//        climbRight.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//
//        climbLeftPID = climbLeft.getClosedLoopController();
//        climbRightPID = climbRight.getClosedLoopController();
//
//        climbLeftSetPoint = getClimberLeftPosition();
//        climbRightSetPoint = getClimberRightPosition();
//
//        setClimbSetPoint(1);
////        setClimberVoltage(-1);
//    }
//
////    public void setClimberVoltage(double volts) {
////        volts = MathUtil.clamp(volts, -12, 12);
////        climbLeft.setVoltage(volts);
////        climbRight.setVoltage(volts);
////    }
//
//    public void setClimbSetPoint(double setPoint) {
//        setPoint = MathUtil.clamp(setPoint, Constants.ClimberConstants.kAnchorHardDeck, Constants.ClimberConstants.kWinchCeiling);
//        climbRightPID.setSetpoint(setPoint, SparkBase.ControlType.kVoltage);
//        climbLeftPID.setSetpoint(setPoint, SparkBase.ControlType.kVoltage);
//    }
//
//    public double getClimbLeftSetPoint() {
//        return climbLeftSetPoint;
//    }
//
//    public double getClimbRightSetPoint() {
//        return climbRightSetPoint;
//    }
//
//    public double getClimbLeftVoltage() {
//        return climbLeft.getBusVoltage();
//    }
//
//    public double getClimbRightVoltage() {
//        return climbLeft.getBusVoltage();
//    }
//
//    public double getClimberLeftPosition() {
//        return climbLeft.getEncoder().getPosition();
//    }
//
//    public double getClimberRightPosition() {
//        return climbRight.getEncoder().getPosition();
//    }
}