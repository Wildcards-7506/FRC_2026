package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

public class LED extends SubsystemBase {

    public enum ColorType {
        SOLID, FLASH, RAINBOW, ALLIANCE_FLOW, STREAMER
    }

    private final AddressableLED ledString;
    private final AddressableLEDBuffer ledBuffer;
    private final int bufferSize;

    private ColorType colorType = ColorType.SOLID;
    private int[][] colors = {{0, 0, 0}};
    private int colorIndex = 0;
    private double lastSwitchTime = 0;

    private static final double SOLID_SWITCH_INTERVAL = 1.0;
    private static final double FLASH_RATE = 0.25;

    private int rainbowFirstPixelHue = 0;

    public int streamerBrightness = 0;
    private int streamerPosition = 0;
    private int streamerDirection = -1;
    private int streamerStallCounter = 0;

    public LED(int pwmPort, int bufferSize) {
        this.bufferSize = bufferSize;
        ledString = new AddressableLED(pwmPort);
        ledBuffer = new AddressableLEDBuffer(bufferSize);
        ledString.setLength(bufferSize);
        ledString.setData(ledBuffer);
        ledString.start();
    }

    public void setColorType(ColorType type) {
        colorType = type;
    }

    public void setColor(int r, int g, int b) {
        colors = new int[][]{rgbToHsv(r, g, b)};
        colorIndex = 0;
    }

    public void setColors(int[]... rgbColors) {
        colors = new int[rgbColors.length][];
        for (int i = 0; i < rgbColors.length; i++) {
            colors[i] = rgbToHsv(rgbColors[i][0], rgbColors[i][1], rgbColors[i][2]);
        }
        colorIndex = 0;
    }

    @Override
    public void periodic() {
        switch (colorType) {
            case SOLID -> runSolid();
            case FLASH -> runFlash();
            case RAINBOW -> rainbow();
            case ALLIANCE_FLOW -> allianceFlow();
            case STREAMER -> streamer();
        }
    }

    private void runSolid() {
        if (colors.length > 1 && Timer.getFPGATimestamp() - lastSwitchTime > SOLID_SWITCH_INTERVAL) {
            colorIndex = (colorIndex + 1) % colors.length;
            lastSwitchTime = Timer.getFPGATimestamp();
        }
        int[] hsv = colors[colorIndex];
        fillSolid(hsv[0], hsv[1], hsv[2]);
    }

    private void runFlash() {
        double t = Timer.getFPGATimestamp();
        int[] hsv = colors[(int) (t / FLASH_RATE) % colors.length];
        boolean on = (t % FLASH_RATE) < FLASH_RATE / 2;
        fillSolid(hsv[0], hsv[1], on ? hsv[2] : 0);
    }

    private void rainbow() {
        for (int i = 0; i < bufferSize; i++) {
            ledBuffer.setHSV(i, (rainbowFirstPixelHue + (i * 180 / bufferSize)) % 180, 255, 255);
        }
        rainbowFirstPixelHue = (rainbowFirstPixelHue + 3) % 180;
        pushData();
    }

    private void allianceFlow() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        int baseHue = isRed ? 165 : 90;
        for (int i = 0; i < bufferSize; i++) {
            int hue = (baseHue + (rainbowFirstPixelHue + (i * 45 / bufferSize)) % 45) % 180;
            ledBuffer.setHSV(i, hue, 255, 255);
        }
        rainbowFirstPixelHue = (rainbowFirstPixelHue + 1) % 45;
        pushData();
    }

    private void streamer() {
        if (streamerStallCounter++ % 3 != 0) return;

        if (streamerPosition == bufferSize - 1 || streamerPosition == 0) streamerDirection *= -1;
        streamerPosition += streamerDirection;

        int[] hsv = colors[0];
        fillSolid(hsv[0], 255, streamerBrightness);
        ledBuffer.setHSV(streamerPosition, 0, 0, 255);
        ledBuffer.setHSV(clamp(streamerPosition - streamerDirection), hsv[0], 60, 200);
        ledBuffer.setHSV(clamp(streamerPosition - 2 * streamerDirection), hsv[0], 120, 160);
        pushData();
    }

    private void fillSolid(int h, int s, int v) {
        for (int i = 0; i < bufferSize; i++) ledBuffer.setHSV(i, h, s, v);
        pushData();
    }

    private void pushData() {
        ledString.setData(ledBuffer);
    }

    private int clamp(int value) {
        return Math.max(0, Math.min(bufferSize - 1, value));
    }

    // Stolen because yes
    public static int[] rgbToHsv(int r, int g, int b) {
        float rf = r / 255f, gf = g / 255f, bf = b / 255f;
        float max = Math.max(rf, Math.max(gf, bf));
        float min = Math.min(rf, Math.min(gf, bf));
        float delta = max - min;

        float h = 0;
        if (delta != 0) {
            if (max == rf) h = 60 * (((gf - bf) / delta) % 6);
            else if (max == gf) h = 60 * (((bf - rf) / delta) + 2);
            else h = 60 * (((rf - gf) / delta) + 4);
        }
        if (h < 0) h += 360;

        float s = max == 0 ? 0 : delta / max;

        return new int[]{(int) (h / 2), (int) (s * 255), (int) (max * 255)};
    }
}
