package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import android.util.Log;

/**
 * NeoPixel Driver for FTC Robotics.
 * Manages an internal pixel buffer and handles color format conversion (RGB to GRB)
 * for I2C-based NeoPixel drivers (like the Adafruit SeeSaw, product #5766).
 * * FINAL VERSION incorporating stability fixes, Big-Endian logic, and byte-width correction.
 */
@I2cDeviceType
@DeviceProperties(name = "Adafruit NeoDriver", xmlTag = "NeoDriver")
public class NeoDriver extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    // --- Internal State and Constants ---
    private static final String TAG = "NeoDriver";
    private static final int PIXEL_BYTES = 3;

    // SeeSaw Module Addresses
    public static final int NEOPIXEL_MODULE_ADDR = 0x0E; // The module for NeoPixel commands
    public static final int STATUS_MODULE_ADDR = 0x00;    // The module for chip-wide reset
    private static final int MAX_I2C_WRITE_SIZE = 30; // Max chunk size for data transfer. 30 gives us 10 LEDs

    private short numPixels;
    private byte[] pixelBuffer = new byte[6];

    // Indices for the GRB color order (Standard NeoPixel format)
    private static final int R = 1;
    private static final int G = 0;
    private static final int B = 2;

    // --- User Methods ---

    /**
     * Resets the SeeSaw chip and configures the NeoPixel hardware registers.
     * This MUST be called at the start of your OpMode (in init() or start()).
     */
    public void initializeNeoPixels() {
        try {
            // 1. Reset the chip (Module 0x00, Register 0x7F)
            writeResetCommand();

            // 500ms delay for chip stabilization after reset.
            Thread.sleep(500);

            // 2. Set I2C speed to 400kHz (0x00).
            write8(Register.SPEED, (byte) 0x00);

            // 3. Set the NeoPixel pin to 15 (0x0F).
            write8(Register.PIN, (byte) 0x0F);

        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


    /**
     * Sends the SHOW command (Module 0x0E, Register 0x05) to update the LEDs.
     */
    public void show() {
        writeCommand(Register.SHOW);
        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Sets the color of a single pixel using standard RGB (maps to NeoPixel GRB).
     */
    public void setPixelColor(long pixelIndex, int red, int green, int blue) {
        // For a single pixel, we need to send 6 bytes of data:
        // 1 byte for the write command
        // 2 bytes for start location
        // 3 bytes for the color
        writeShort(Register.BUF_LENGTH, (byte)(6 & 0xFF));

        long startaddress = pixelIndex * 3 - 1;

        if (pixelIndex >= 0) {
            pixelBuffer[0] = 0x04;
            pixelBuffer[1] = (byte) (startaddress >> 8 & 0xFF);
            pixelBuffer[2] = (byte) (startaddress & 0xFF);
            pixelBuffer[3] = (byte) (red & 0xFF);
            pixelBuffer[4] = (byte) (green & 0xFF);
            pixelBuffer[5] = (byte) (blue & 0xFF);
        } else {
            // Using Log.e for clearer error output
            Log.e(TAG, "Pixel index " + pixelIndex + " out of bounds.");
        }

        // write the single pixel buffer to the NeoDriver chip.
        deviceClient.write(NEOPIXEL_MODULE_ADDR, pixelBuffer);
        try {
            // Give the chip time to execute the high-speed data transfer.
            Thread.sleep(200);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    // --- Helper Methods for I2C communication (Implementing the SeeSaw 2-byte protocol) ---

    /** Performs the Software Reset command (Module 0x00, Register 0x7F with data 0xFF). */
    protected void writeResetCommand() {
        byte[] resetData = new byte[]{(byte) Register.SWRST.bVal, (byte) 0xFF};
        deviceClient.write(STATUS_MODULE_ADDR, resetData);
    }

    /** Writes a command byte (Module 0x0E, Register R). */
    protected void writeCommand(final Register reg) {
        byte[] payload = new byte[]{(byte) reg.bVal};
        deviceClient.write(NEOPIXEL_MODULE_ADDR, payload);
    }

    /** * Writes an 8-bit byte value (Module 0x0E, Register R, Data).
     * Used for single-byte registers like PIN and SPEED.
     */
    protected void write8(final Register reg, byte value) {
        byte[] payload = new byte[2];
        payload[0] = (byte) reg.bVal;
        payload[1] = value;

        deviceClient.write(NEOPIXEL_MODULE_ADDR, payload);
    }

    /** * Writes a 16-bit short value (Module 0x0E, Register R, Data High, Data Low).
     * Used ONLY for 16-bit registers like BUF_LENGTH.
     */
    protected void writeShort(final Register reg, short value) {
        byte[] data = new byte[2];
        data[0] = (byte)(value & 0xFF);
        data[1] = (byte)((value>>8) & 0xFF);

        byte[] payload = new byte[1 + data.length];
        payload[0] = (byte) reg.bVal;

        System.arraycopy(data, 0, payload, 1, data.length);

        deviceClient.write(NEOPIXEL_MODULE_ADDR, data);
    }

    // --- Registers and Config Settings ---
    public enum Register{
        PIN(0x01),
        SPEED(0x02),
        BUF_LENGTH(0x03),
        BUF(0x04),
        SHOW(0x05),
        SWRST(0x7F); // Software Reset (in Status Module 0x00)

        public int bVal;

        Register(int bVal){
            this.bVal = bVal;
        }
    }

    // --- Construction and Initialization ---
    // Default I2C address for the Adafruit NeoDriver
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x60);

    public NeoDriver(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Adafruit;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        // Only perform the minimal setup required by the SDK for the device to be recognized.
        // All device-specific configuration (Reset, Pin, Speed) is now in initializeNeoPixels().
        return true;
    }

    @Override
    public String getDeviceName()
    {
        return "Adafruit NeoDriver - I2C to NeoPixel Driver";
    }
}