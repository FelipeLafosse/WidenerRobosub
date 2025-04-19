import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from smbus2 import SMBus
import numpy as np
from scipy.fft import fft
from scipy.signal import butter, filtfilt
import time

# === ADC and Sampling Settings ===
ADC_ADDRESS = 0x48  # ADS1115 default I2C address
SAMPLE_RATE = 850  # 860 is the Max sampling rate for ADS1115
BUFFER_SIZE = 256  # Number of samples in each buffer
AMPLITUDE_THRESHOLD = 0.01  # Set based on observed environment

# === Initialize I2C bus ===
bus = SMBus(1)

# === Bandpass Filter for 400 Hz Detection ===
def bandpass_filter(signal, lowcut=390, highcut=400, fs=SAMPLE_RATE, order=4):
    nyquist = 0.5 * fs
    highcut = min(highcut, nyquist - 1)  # keep highcut safely below Nyquist
    low = lowcut / nyquist
    high = highcut / nyquist
    b, a = butter(order, [low, high], btype='band')
    return filtfilt(b, a, signal)


# === Proper 16-bit read from ADS1115 ===
def read_adc(channel=0):
    """
    Read 16-bit voltage from ADS1115 on the selected channel (0–3).
    """
    if channel < 0 or channel > 3:
        return None

    CONFIG_REG = 0x01
    CONVERSION_REG = 0x00

    # ADS1115 config for single-shot conversion
    config = 0x8000  # Start single conversion
    config |= 0x4000  # Single-ended input
    config |= (channel & 0x03) << 12  # Channel select
    config |= 0x0400  # ±4.096V range
    config |= 0x00E0  # 860 SPS
    config |= 0x0003  # Disable comparator

    bus.write_i2c_block_data(ADC_ADDRESS, CONFIG_REG, [(config >> 8) & 0xFF, config & 0xFF])
    time.sleep(1 / SAMPLE_RATE + 0.001)  # Wait for conversion

    result = bus.read_i2c_block_data(ADC_ADDRESS, CONVERSION_REG, 2)
    raw = (result[0] << 8) | result[1]
    if raw > 0x7FFF:
        raw -= 0x10000

    voltage = raw * (4.096 / 32768.0)  # Convert to voltage using ±4.096V reference
    return voltage

# === Frequency Detection ===
def detect_dominant_frequency(signal, threshold=AMPLITUDE_THRESHOLD):
    filtered = bandpass_filter(signal)
    filtered = filtered - np.mean(filtered)
    filtered/= np.max(np.abs(filtered) + 1e-6)
    fft_result = fft(filtered)
    magnitude = np.abs(fft_result)
    freqs = np.fft.fftfreq(len(signal), 1 / SAMPLE_RATE)

    pos_indices = np.where(freqs > 0)
    pos_freqs = freqs[pos_indices]
    pos_magnitude = magnitude[pos_indices]

    peak_index = np.argmax(pos_magnitude)
    peak_freq = pos_freqs[peak_index]
    peak_amp = pos_magnitude[peak_index]

    print(f"Peak Frequency: {peak_freq:.2f} Hz | Amplitude: {peak_amp:.2f}")

    if peak_amp > threshold:
        return True, peak_freq, peak_amp
    return False, None, 0

# === ROS2 Node ===
class HydrophoneNode(Node):
    def __init__(self):
        super().__init__('hydrophone_node')
        self.publisher_ = self.create_publisher(Float32, 'ping_detected', 10)
        self.timer = self.create_timer(BUFFER_SIZE / SAMPLE_RATE, self.timer_callback)
        self.buffer = np.zeros(BUFFER_SIZE)
        self.get_logger().info("Hydrophone node initialized with ADS1115.")

    def timer_callback(self):
        for i in range(BUFFER_SIZE):
            voltage = read_adc()
            self.buffer[i] = voltage if voltage is not None else 0

        detected, freq, amp = detect_dominant_frequency(self.buffer)
        if detected:
            self.get_logger().info(f"Detected {freq:.2f} Hz | Amplitude: {amp:.2f}")
            msg = Float32()
            msg.data = float(freq)
            self.publisher_.publish(msg)

# === Main ===
def main(args=None):
    rclpy.init(args=args)
    node = HydrophoneNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down hydrophone node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
