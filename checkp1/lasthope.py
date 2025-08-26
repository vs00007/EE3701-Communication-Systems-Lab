#!/usr/bin/env python3
"""
Simple ADALM-PLUTO QAM Transceiver Implementation
Based on the GNU Radio flowgraph shown in the image
"""

import numpy as np
import adi
import threading
import time
from scipy import signal
import matplotlib.pyplot as plt

class PlutoQAMTransceiver:
    def __init__(self, uri="ip:192.168.2.1"):
        """
        Initialize ADALM-PLUTO transceiver
        """
        # SDR parameters
        self.sample_rate = 2000000  # 2 MHz
        self.center_freq = 2400000000  # 2.4 GHz
        self.tx_gain = -10  # dBm
        self.rx_gain = 64   # dB
        
        # Modulation parameters
        self.constellation_size = 16  # 16-QAM
        self.symbol_rate = 250000     # 250 kSps
        self.sps = 8  # samples per symbol
        
        # Initialize PLUTO
        try:
            self.sdr = adi.Pluto(uri)
            self.setup_sdr()
            print("ADALM-PLUTO initialized successfully")
        except Exception as e:
            print(f"Error initializing PLUTO: {e}")
            self.sdr = None
    
    def setup_sdr(self):
        """Configure SDR parameters"""
        # TX configuration
        self.sdr.tx_lo = self.center_freq
        self.sdr.tx_hardwaregain_chan0 = self.tx_gain
        self.sdr.sample_rate = self.sample_rate
        
        # RX configuration  
        self.sdr.rx_lo = self.center_freq
        self.sdr.gain_control_mode_chan0 = 'manual'
        self.sdr.rx_hardwaregain_chan0 = self.rx_gain
        self.sdr.rx_buffer_size = 4096
    
    def generate_qam16_constellation(self):
        """Generate 16-QAM constellation points"""
        constellation = []
        for i in range(4):
            for q in range(4):
                # Normalize to unit average power
                real = (2*i - 3) / np.sqrt(10)
                imag = (2*q - 3) / np.sqrt(10)
                constellation.append(complex(real, imag))
        return np.array(constellation)
    
    def bits_to_symbols(self, bits):
        """Convert bit stream to QAM symbols"""
        constellation = self.generate_qam16_constellation()
        symbols = []
        
        # Group bits into 4-bit chunks for 16-QAM
        for i in range(0, len(bits), 4):
            if i + 3 < len(bits):
                # Convert 4 bits to decimal index
                symbol_index = (bits[i] << 3) + (bits[i+1] << 2) + (bits[i+2] << 1) + bits[i+3]
                symbols.append(constellation[symbol_index])
        
        return np.array(symbols)
    
    def symbols_to_bits(self, symbols):
        """Demodulate QAM symbols back to bits"""
        constellation = self.generate_qam16_constellation()
        bits = []
        
        for symbol in symbols:
            # Find closest constellation point
            distances = np.abs(constellation - symbol)
            closest_index = np.argmin(distances)
            
            # Convert index back to 4 bits
            bits.extend([
                (closest_index >> 3) & 1,
                (closest_index >> 2) & 1, 
                (closest_index >> 1) & 1,
                closest_index & 1
            ])
        
        return bits
    
    def pulse_shape(self, symbols):
        """Apply root raised cosine pulse shaping"""
        # Simple upsampling and filtering
        upsampled = np.zeros(len(symbols) * self.sps, dtype=complex)
        upsampled[::self.sps] = symbols
        
        # Root raised cosine filter (simplified)
        alpha = 0.35  # roll-off factor
        span = 6
        t = np.arange(-span*self.sps//2, span*self.sps//2) / self.sps
        
        # RRC filter impulse response
        rrc = np.sinc(t) * np.cos(np.pi * alpha * t) / (1 - (2 * alpha * t)**2)
        rrc[np.isnan(rrc)] = alpha / 4  # Handle singularities
        rrc = rrc / np.sqrt(np.sum(rrc**2))
        
        # Filter the signal
        filtered = signal.convolve(upsampled, rrc, mode='same')
        
        return filtered
    
    def generate_test_data(self, num_bytes=100):
        """Generate random test data"""
        # Create random bits
        data_bits = np.random.randint(0, 2, num_bytes * 8)
        return data_bits
    
    def transmit(self, bits):
        """Transmit bit stream"""
        if self.sdr is None:
            print("SDR not initialized")
            return
        
        # Convert bits to symbols
        symbols = self.bits_to_symbols(bits)
        
        # Pulse shaping
        tx_signal = self.pulse_shape(symbols)
        
        # Scale for transmission
        tx_signal = tx_signal * 0.5  # Prevent clipping
        
        # Convert to int16 for PLUTO
        tx_samples = (tx_signal * 2**14).astype(np.complex64)
        
        try:
            # Transmit
            self.sdr.tx(tx_samples)
            print(f"Transmitted {len(bits)} bits ({len(symbols)} symbols)")
        except Exception as e:
            print(f"Transmission error: {e}")
    
    def receive(self, num_samples=4096):
        """Receive and process signal"""
        if self.sdr is None:
            print("SDR not initialized")
            return None
        
        try:
            # Receive samples
            rx_samples = self.sdr.rx()
            
            # Basic carrier recovery (simplified)
            # In practice, you'd need proper synchronization
            
            # Matched filtering (using same RRC filter)
            alpha = 0.35
            span = 6
            t = np.arange(-span*self.sps//2, span*self.sps//2) / self.sps
            rrc = np.sinc(t) * np.cos(np.pi * alpha * t) / (1 - (2 * alpha * t)**2)
            rrc[np.isnan(rrc)] = alpha / 4
            rrc = rrc / np.sqrt(np.sum(rrc**2))
            
            filtered = signal.convolve(rx_samples, rrc, mode='same')
            
            # Symbol timing recovery (simplified - just downsample)
            symbols = filtered[::self.sps]
            
            # Remove some edge symbols to avoid filter artifacts
            if len(symbols) > 20:
                symbols = symbols[10:-10]
            
            # Demodulate symbols to bits
            bits = self.symbols_to_bits(symbols)
            
            print(f"Received {len(symbols)} symbols ({len(bits)} bits)")
            return bits, symbols
            
        except Exception as e:
            print(f"Reception error: {e}")
            return None, None
    
    def plot_constellation(self, symbols):
        """Plot received constellation"""
        plt.figure(figsize=(8, 6))
        plt.scatter(symbols.real, symbols.imag, alpha=0.7)
        
        # Plot ideal constellation points
        ideal = self.generate_qam16_constellation()
        plt.scatter(ideal.real, ideal.imag, c='red', marker='x', s=100)
        
        plt.xlabel('In-phase')
        plt.ylabel('Quadrature')
        plt.title('16-QAM Constellation')
        plt.grid(True)
        plt.axis('equal')
        plt.show()
    
    def loopback_test(self):
        """Simple loopback test"""
        print("Starting loopback test...")
        
        # Generate test data
        test_bits = self.generate_test_data(20)  # 20 bytes
        print(f"Test data: {test_bits[:32]}...")  # Show first 32 bits
        
        # Start receiver in separate thread
        def receiver_thread():
            time.sleep(0.5)  # Let transmitter start first
            rx_bits, rx_symbols = self.receive()
            if rx_bits is not None:
                # Compare first bits (accounting for sync issues)
                min_len = min(len(test_bits), len(rx_bits))
                if min_len > 32:
                    errors = sum(a != b for a, b in zip(test_bits[:32], rx_bits[:32]))
                    print(f"Bit errors in first 32 bits: {errors}/32")
                
                # Plot constellation
                if rx_symbols is not None:
                    self.plot_constellation(rx_symbols)
        
        # Start receiver
        rx_thread = threading.Thread(target=receiver_thread)
        rx_thread.start()
        
        # Transmit test data
        self.transmit(test_bits)
        
        # Wait for receiver
        rx_thread.join()
    
    def close(self):
        """Clean up resources"""
        if self.sdr:
            del self.sdr

def main():
    """Main function for testing"""
    # Create transceiver instance
    # Change URI if your PLUTO has different IP
    transceiver = PlutoQAMTransceiver("ip:192.168.2.1")
    
    try:
        # Run loopback test
        transceiver.loopback_test()
        
        # You can also transmit custom data
        # custom_bits = [1,0,1,1,0,0,1,0] * 10  # repeat pattern
        # transceiver.transmit(custom_bits)
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        transceiver.close()

if __name__ == "__main__":
    main()