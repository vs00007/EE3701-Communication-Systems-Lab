function [RX_freq, RX_freq_eq, no_equalizer, no_pilot, demodQPSK, demodBPSK, h_est, H_est_recon] = process_payload(RX_payload_time, Nfft, Lch, pilot_idx, pilot_symbols)
% Process one payload OFDM time symbol (with CP present)
% RX_payload_time : [1x80] time-domain symbol including CP
% Returns frequency symbol, equalized symbol, selected data carriers, demapped symbols and channel estimates

% Remove cyclic prefix (assumes CP length = 16)
RX_payload_no_CP = RX_payload_time(17:end);            % [1x64]
RX_freq = fftshift(fft(RX_payload_no_CP));            % [1x64]

% Extract pilots and form LS problem
Yp = RX_freq(pilot_idx).';                             % (4x1)
Xp = pilot_symbols;                                    % (4x1)

% Build pilot matrix F (len(pilot_idx) x Lch)
% vectorized Vandermonde
% F = exp(-1j*2*pi*((pilot_idx(:)-1) * (0:(Lch-1))) / Nfft); % (4 x Lch)

F = zeros(length(pilot_idx), Lch);
for ii = 1:length(pilot_idx)
    for l = 1:Lch
        F(ii,l) = exp(-1j*2*pi*(pilot_idx(ii)-1)*(l-1)/Nfft);
    end
end


% LS estimate of CIR taps for payload1
% avoid explicit pinv when possible; use backslash as stable LS
h_est = F \ (Yp ./ Xp);   % (Lch x 1)

% Reconstruct frequency response for all 64 subcarriers
H_est_recon = zeros(Nfft,1);
for k = 1:Nfft
    H_est_recon(k) = sum(h_est.' .* exp(-1j*2*pi*(k-1)*(0:Lch-1)/Nfft));
end
H_est_recon = H_est_recon.'; % make row to match earlier H_est shape if needed

% Equalize payload1 using reconstructed H
RX_freq_eq = RX_freq ./ H_est_recon; % [1x64]            % [1x64]

% De-mapping: select data carriers (same mapping as original code)
no_equalizer = [RX_freq(7:11), RX_freq(13:25), RX_freq(27:32), RX_freq(34:39), RX_freq(41:53), RX_freq(55:59)];   % [1x48]
no_pilot     = [RX_freq_eq(7:11), RX_freq_eq(13:25), RX_freq_eq(27:32), RX_freq_eq(34:39), RX_freq_eq(41:53), RX_freq_eq(55:59)]; % [1x48]

% Demodulation
demodQPSK = pskdemod(no_pilot, 4, pi/4);               % [1x48]
demodBPSK = pskdemod(no_pilot, 2);                     % [1x48]
end