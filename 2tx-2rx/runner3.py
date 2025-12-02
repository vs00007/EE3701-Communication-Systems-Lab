import adi
import numpy as np
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QLabel
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg

# -----------------------------------
# Setup Parameters
# -----------------------------------
samp_rate = 2e6
rx_lo = 2.3e9
rx_mode = "manual"
rx_gain0 = 40
rx_gain1 = 40
tx_lo = rx_lo
tx_gain = -3
fc0 = int(200e3)
phase_cal = 0

NumSamples = 2**12   # RX buffer size
N_tx       = 2**12   # TX chirp size

# -----------------------------------
# Antenna separation information
# -----------------------------------
d_wavelength = 0.5
wavelength = 3E8 / rx_lo
d = d_wavelength * wavelength
print("Set distance between Rx Antennas to", int(d * 1000), "mm")

# -----------------------------------
# Create SDR
# -----------------------------------
sdr = adi.ad9361(uri='ip:192.168.2.1')

# RX configuration
sdr.rx_enabled_channels = [0, 1]
sdr.sample_rate = int(samp_rate)
sdr.rx_rf_bandwidth = int(fc0 * 3)
sdr.rx_lo = int(rx_lo)
sdr.gain_control_mode = rx_mode
sdr.rx_hardwaregain_chan0 = int(rx_gain0)
sdr.rx_hardwaregain_chan1 = int(rx_gain1)
sdr.rx_buffer_size = int(NumSamples)
sdr._rxadc.set_kernel_buffers_count(1)

# TX configuration
sdr.tx_rf_bandwidth = int(fc0 * 3)
sdr.tx_lo = int(rx_lo)
sdr.tx_cyclic_buffer = True
sdr.tx_hardwaregain_chan0 = int(tx_gain)
sdr.tx_hardwaregain_chan1 = int(-88)
sdr.tx_buffer_size = int(N_tx)

# -----------------------------------
# Generate Chirps
# -----------------------------------
fs = int(sdr.sample_rate)
ts = 1 / float(fs)

B = 200e3
T_chirp = N_tx * ts
f1 = B // 5    # bit=1 chirp
f2 = B // 5    # bit=0 chirp
k  = B / T_chirp

def generate_chirp(f_start, N_tx, ts, k):
    t = np.arange(0, N_tx * ts / 4, ts / 4)
    phi = 2 * np.pi * (f_start * t + 0.5 * k * t**2)
    i = np.cos(phi) * 2**14
    q = np.sin(phi) * 2**14
    return i + 1j * q

chirp1 = generate_chirp(f1, N_tx, ts, k)  # bit=1
chirp2 = generate_chirp(f2, N_tx, ts, -k)  # bit=0

# Choose which bit to transmit (0 or 1)
TX_BIT = 0
if TX_BIT == 1:
    sdr.tx([chirp1, chirp1])
    print(f"Transmitting bit = 1 (chirp at {f1} Hz)")
else:
    sdr.tx([chirp2, chirp2])
    print(f"Transmitting bit = 0 (chirp at {f2} Hz)")

# Assign frequency bins for tracking
xf = np.fft.fftfreq(NumSamples, ts)
xf = np.fft.fftshift(xf) / 1e6
signal_start = int(NumSamples * (samp_rate/2 + fc0/2) / samp_rate)
signal_end = int(NumSamples * (samp_rate/2 + fc0*2) / samp_rate)

# -----------------------------------
# FFT-based Matched Filter Utilities
# -----------------------------------
def next_pow2(x):
    return 1 << (int(x) - 1).bit_length()

def prepare_template(template):
    template = np.asarray(template).astype(np.complex128)
    energy = np.vdot(template, template).real
    template = template / np.sqrt(energy)
    return template, np.conjugate(template[::-1])

def precompute_filter_fft(templ_rev_conj, rx_len):
    L = templ_rev_conj.size
    nfft = next_pow2(rx_len + L - 1)
    H = np.fft.fft(np.pad(templ_rev_conj, (0, nfft - L)))
    return nfft, H

def correlate_fft(x, H, nfft):
    X = np.fft.fft(np.pad(x, (0, nfft - x.size)))
    corr = np.fft.ifft(X * H)
    return corr

# -----------------------------------
# Prepare chirp templates
# -----------------------------------
templ1, templ1_rc = prepare_template(chirp1)
templ2, templ2_rc = prepare_template(chirp2)

templ1_len = len(templ1)
templ2_len = len(templ2)

nfft1, H1 = precompute_filter_fft(templ1_rc, NumSamples)
nfft2, H2 = precompute_filter_fft(templ2_rc, NumSamples)

# -----------------------------------
# Monopulse Angle Detection Functions
# -----------------------------------
def calcTheta(phase):
    arcsin_arg = np.deg2rad(phase) * 3E8 / (2 * np.pi * rx_lo * d)
    arcsin_arg = max(min(1, arcsin_arg), -1)
    calc_theta = np.rad2deg(np.arcsin(arcsin_arg))
    return calc_theta

def dbfs(raw_data):
    NumSamples = len(raw_data)
    win = np.hamming(NumSamples)
    y = raw_data * win
    s_fft = np.fft.fft(y) / np.sum(win)
    s_shift = np.fft.fftshift(s_fft)
    s_dbfs = 20 * np.log10(np.maximum(np.abs(s_shift), 1e-10) / (2**11))
    return s_shift, s_dbfs

def monopulse_angle(array1, array2):
    sum_delta_correlation = np.correlate(array1[signal_start:signal_end], 
                                         array2[signal_start:signal_end], 'valid')
    angle_diff = np.angle(sum_delta_correlation)
    return angle_diff

def Tracking(Rx_0, Rx_1, last_delay):
    delayed_Rx_1 = Rx_1 * np.exp(1j * np.deg2rad(last_delay + phase_cal))
    delayed_sum = Rx_0 + delayed_Rx_1
    delayed_delta = Rx_0 - delayed_Rx_1
    delayed_sum_fft, delayed_sum_dbfs = dbfs(delayed_sum)
    delayed_delta_fft, delayed_delta_dbfs = dbfs(delayed_delta)
    mono_angle = monopulse_angle(delayed_sum_fft, delayed_delta_fft)
    phase_step = 1
    if np.sign(mono_angle) > 0:
        new_delay = last_delay - phase_step
    else:
        new_delay = last_delay + phase_step
    return new_delay

def scan_for_DOA(Rx_0, Rx_1):
    peak_sum = []
    delay_phases = np.arange(-180, 180, 2)
    
    for phase_delay in delay_phases:   
        delayed_Rx_1 = Rx_1 * np.exp(1j * np.deg2rad(phase_delay + phase_cal))
        delayed_sum = Rx_0 + delayed_Rx_1
        delayed_sum_fft, delayed_sum_dbfs = dbfs(delayed_sum)
        peak_sum.append(np.max(delayed_sum_dbfs))
        
    peak_dbfs = np.max(peak_sum)
    peak_delay_index = np.where(peak_sum == peak_dbfs)
    peak_delay = delay_phases[peak_delay_index[0][0]]
    steer_angle = int(calcTheta(peak_delay))
    
    return peak_dbfs, peak_delay, steer_angle

# -----------------------------------
# Bit Detector with Per-Channel Correlation
# -----------------------------------
def detect_bit_with_channels(ch0, ch1):
    """Detect bit and return correlations for both channels separately"""
    # Channel 0
    corr1_ch0 = correlate_fft(ch0, H1, nfft1)[:ch0.size + templ1_len - 1]
    corr2_ch0 = correlate_fft(ch0, H2, nfft2)[:ch0.size + templ2_len - 1]
    p1_ch0 = np.max(np.abs(corr1_ch0))
    p2_ch0 = np.max(np.abs(corr2_ch0))
    
    # Channel 1
    corr1_ch1 = correlate_fft(ch1, H1, nfft1)[:ch1.size + templ1_len - 1]
    corr2_ch1 = correlate_fft(ch1, H2, nfft2)[:ch1.size + templ2_len - 1]
    p1_ch1 = np.max(np.abs(corr1_ch1))
    p2_ch1 = np.max(np.abs(corr2_ch1))
    
    # Combine for bit decision
    p1_total = p1_ch0 + p1_ch1
    p2_total = p2_ch0 + p2_ch1
    
    bit = 1 if p1_total > p2_total else 0
    
    return bit, p1_total, p2_total

# -----------------------------------
# Calibrate and Initial Scan
# -----------------------------------
print("Calibrating SDR...")
for i in range(20):  
    data = sdr.rx()

print("Scanning for initial DOA...")
data = sdr.rx()
peak_dbfs, peak_delay, steer_angle = scan_for_DOA(data[0], data[1])
delay = peak_delay
print(f"Initial angle detected: {steer_angle}°")

# -----------------------------------
# PyQt GUI Application
# -----------------------------------
class MonopulseGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Chirp Detection & Monopulse Tracking")
        self.setGeometry(100, 100, 1400, 900)
        
        # Data buffers
        self.max_points = 500
        self.time_data = []
        self.angle_data = []
        self.corr1_data = []
        self.corr2_data = []
        self.bit_data = []
        
        self.count = 0
        self.running = False
        self.delay = delay
        
        # Setup UI
        self.setup_ui()
        
        # Timer for updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        
    def setup_ui(self):
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Info labels
        info_layout = QHBoxLayout()
        self.info_label = QLabel(f"TX Bit: {TX_BIT} | Wavelength: {wavelength*1000:.2f} mm | Antenna Spacing: {d*1000:.2f} mm")
        self.info_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        info_layout.addWidget(self.info_label)
        main_layout.addLayout(info_layout)
        
        # Status labels
        status_layout = QHBoxLayout()
        self.status_label = QLabel("Status: Stopped")
        self.status_label.setStyleSheet("font-size: 12px;")
        self.bit_label = QLabel("Detected Bit: --")
        self.bit_label.setStyleSheet("font-size: 12px; font-weight: bold;")
        self.angle_label = QLabel(f"Angle: {steer_angle:+7.2f}°")
        self.angle_label.setStyleSheet("font-size: 12px; font-weight: bold;")
        self.delay_label = QLabel(f"Phase Delay: {self.delay:.1f}°")
        self.delay_label.setStyleSheet("font-size: 12px;")
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(self.bit_label)
        status_layout.addWidget(self.angle_label)
        status_layout.addWidget(self.delay_label)
        status_layout.addStretch()
        main_layout.addLayout(status_layout)
        
        # Plot widgets
        pg.setConfigOptions(antialias=True)
        
        # Angle plot
        self.angle_plot = pg.PlotWidget(title="Angle of Arrival (degrees)")
        self.angle_plot.setLabel('left', 'Angle', units='°')
        self.angle_plot.setLabel('bottom', 'Sample')
        self.angle_plot.showGrid(x=True, y=True)
        self.angle_curve = self.angle_plot.plot(pen=pg.mkPen(color='c', width=2))
        main_layout.addWidget(self.angle_plot)
        
        # Correlation plot
        self.corr_plot = pg.PlotWidget(title="Correlation Strength")
        self.corr_plot.setLabel('left', 'Correlation')
        self.corr_plot.setLabel('bottom', 'Sample')
        self.corr_plot.showGrid(x=True, y=True)
        self.corr1_curve = self.corr_plot.plot(pen=pg.mkPen(color='g', width=2), name='Bit 1')
        self.corr2_curve = self.corr_plot.plot(pen=pg.mkPen(color='r', width=2), name='Bit 0')
        self.corr_plot.addLegend()
        main_layout.addWidget(self.corr_plot)
        
        # Bit history plot
        self.bit_plot = pg.PlotWidget(title="Detected Bit History")
        self.bit_plot.setLabel('left', 'Bit Value')
        self.bit_plot.setLabel('bottom', 'Sample')
        self.bit_plot.setYRange(-0.2, 1.2)
        self.bit_plot.showGrid(x=True, y=True)
        self.bit_curve = self.bit_plot.plot(pen=None, symbol='o', symbolPen=pg.mkPen(color='y', width=2), 
                                            symbolBrush='y', symbolSize=6)
        main_layout.addWidget(self.bit_plot)
        
        # Control buttons
        button_layout = QHBoxLayout()
        self.start_button = QPushButton("Start")
        self.start_button.clicked.connect(self.start_acquisition)
        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(self.stop_acquisition)
        self.stop_button.setEnabled(False)
        self.clear_button = QPushButton("Clear")
        self.clear_button.clicked.connect(self.clear_plots)
        self.rescan_button = QPushButton("Rescan DOA")
        self.rescan_button.clicked.connect(self.rescan_doa)
        
        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.stop_button)
        button_layout.addWidget(self.clear_button)
        button_layout.addWidget(self.rescan_button)
        button_layout.addStretch()
        main_layout.addLayout(button_layout)
        
    def start_acquisition(self):
        self.running = True
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.status_label.setText("Status: Running")
        self.timer.start(0)  # Update as fast as possible
        
    def stop_acquisition(self):
        self.running = False
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.status_label.setText("Status: Stopped")
        self.timer.stop()
        
    def clear_plots(self):
        self.time_data.clear()
        self.angle_data.clear()
        self.corr1_data.clear()
        self.corr2_data.clear()
        self.bit_data.clear()
        self.count = 0
        self.update_plots()
        
    def rescan_doa(self):
        print("Rescanning for DOA...")
        data = sdr.rx()
        peak_dbfs, peak_delay, steer_angle = scan_for_DOA(data[0], data[1])
        self.delay = peak_delay
        self.angle_label.setText(f"Angle: {steer_angle:+7.2f}°")
        self.delay_label.setText(f"Phase Delay: {self.delay:.1f}°")
        print(f"New angle detected: {steer_angle}°, Phase delay: {self.delay:.1f}°")
        
    def update_data(self):
        try:
            # Get RX data
            rx = sdr.rx()
            ch0 = rx[0]
            ch1 = rx[1]
            
            # Size correction
            if len(ch0) < NumSamples:
                ch0 = np.pad(ch0, (0, NumSamples - len(ch0)))
            elif len(ch0) > NumSamples:
                ch0 = ch0[:NumSamples]
                
            if len(ch1) < NumSamples:
                ch1 = np.pad(ch1, (0, NumSamples - len(ch1)))
            elif len(ch1) > NumSamples:
                ch1 = ch1[:NumSamples]
            
            # Detect bit
            bit, p1, p2 = detect_bit_with_channels(ch0, ch1)
            
            # Track angle using monopulse
            self.delay = Tracking(ch0, ch1, self.delay)
            angle = calcTheta(self.delay)
            
            # Store data
            self.time_data.append(self.count)
            self.angle_data.append(angle)
            self.corr1_data.append(p1)
            self.corr2_data.append(p2)
            self.bit_data.append(bit)
            
            # Limit buffer size
            if len(self.time_data) > self.max_points:
                self.time_data.pop(0)
                self.angle_data.pop(0)
                self.corr1_data.pop(0)
                self.corr2_data.pop(0)
                self.bit_data.pop(0)
            
            # Update labels
            self.bit_label.setText(f"Detected Bit: {bit}")
            self.angle_label.setText(f"Angle: {angle:+7.2f}°")
            self.delay_label.setText(f"Phase Delay: {self.delay:.1f}°")
            
            # Update plots
            self.update_plots()
            
            self.count += 1
            
        except Exception as e:
            print(f"Error in update_data: {e}")
            self.stop_acquisition()
    
    def update_plots(self):
        if len(self.time_data) > 0:
            self.angle_curve.setData(self.time_data, self.angle_data)
            self.corr1_curve.setData(self.time_data, self.corr1_data)
            self.corr2_curve.setData(self.time_data, self.corr2_data)
            self.bit_curve.setData(self.time_data, self.bit_data)
    
    def closeEvent(self, event):
        self.stop_acquisition()
        sdr.tx_destroy_buffer()
        event.accept()

# -----------------------------------
# Main Application
# -----------------------------------
if __name__ == '__main__':
    print("Starting combined chirp detection & monopulse tracking...")
    print(f"Wavelength: {wavelength*1000:.2f} mm")
    print(f"Antenna spacing: {d*1000:.2f} mm")
    print("-" * 60)
    
    app = QApplication(sys.argv)
    window = MonopulseGUI()
    window.show()
    sys.exit(app.exec_())