import adi
import numpy as np
import threading
import time
import random

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
tracking_length = 1000

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
sdr.tx_cyclic_buffer = False
sdr.tx_hardwaregain_chan0 = int(tx_gain)
sdr.tx_hardwaregain_chan1 = int(-88)
sdr.tx_buffer_size = int(N_tx)

# -----------------------------------
# Generate Chirps
# -----------------------------------
fs = int(sdr.sample_rate)
ts = 1 / float(fs)

t_tx = np.arange(0, N_tx * ts / 4, ts / 4)

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

chirp1 = generate_chirp(f1, N_tx, ts, k)
chirp2 = generate_chirp(f2, N_tx, ts,-k)

# -----------------------------------
# Random TX Switching Thread
# -----------------------------------
TX_BITRATE = 20    # bits per second (adjustable)
TX_PERIOD = 1 / TX_BITRATE

running_tx = True

def tx_thread():
    """Continuously switch TX buffer between chirp1 and chirp2."""
    while running_tx:
        bit = random.randint(0, 1)     # random bit

        if bit == 1:
            sdr.tx([chirp1, chirp1])
        else:
            sdr.tx([chirp2, chirp2])

        print(f"[TX] Sent bit {bit}")
        time.sleep(TX_PERIOD)

# Start background TX thread
thr = threading.Thread(target=tx_thread)
thr.daemon = True
thr.start()

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

# Channel combiner
def combine_channels(rx_list):
    minlen = min(len(x) for x in rx_list)
    out = np.zeros(minlen, dtype=np.complex128)
    for ch in rx_list:
        out += ch[:minlen]
    return out

# -----------------------------------
# Minimal Bit Detector
# -----------------------------------
def detect_bit(rx_buf):
    corr1 = correlate_fft(rx_buf, H1, nfft1)[:rx_buf.size + templ1_len - 1]
    corr2 = correlate_fft(rx_buf, H2, nfft2)[:rx_buf.size + templ2_len - 1]

    p1 = np.max(np.abs(corr1))
    p2 = np.max(np.abs(corr2))

    bit = 1 if p1 > p2 else 0
    return bit, p1, p2

# -----------------------------------
# Main Receive Loop
# -----------------------------------
print("Starting real-time chirp detector...")

count = 0
try:
    while True:
        rx = sdr.rx()

        if len(rx) == 2:
            rx_buf = combine_channels(rx)
        else:
            rx_buf = rx[0]

        if len(rx_buf) < NumSamples:
            rx_buf = np.pad(rx_buf, (0, NumSamples - len(rx_buf)))
        elif len(rx_buf) > NumSamples:
            rx_buf = rx_buf[:NumSamples]

        bit, c1, c2 = detect_bit(rx_buf)
        print(f"[RX] Bit={bit}   Corr1={c1:.3f}   Corr2={c2:.3f}")

        count += 1
        if count > tracking_length:
            break

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    running_tx = False
    thr.join(timeout=1)
