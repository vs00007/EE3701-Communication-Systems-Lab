function rxOFDM
    fs = 1e6; centerFreq = 0.5e9;
    Nfft = 64; Ncp = 16; M = 4; numOFDMSym = 20;
    pilotIdx = [12;26;40;54];

    % OFDM demodulator
    ofdmDemod = comm.OFDMDemodulator( ...
        'FFTLength', Nfft, ...
        'NumGuardBandCarriers', [6;5], ...
        'InsertDCNull', true, ...
        'PilotOutputPort', true, ...
        'PilotCarrierIndices', pilotIdx, ...
        'CyclicPrefixLength', Ncp, ...
        'NumSymbols', numOFDMSym);

    % Pluto RX
    rx = sdrrx('Pluto', 'CenterFrequency', centerFreq, ...
        'BasebandSampleRate', fs, 'SamplesPerFrame', 16384, ...
        'OutputDataType', 'double');

    fprintf("RX running...\n");
    L = Nfft/2; % preamble half-length

    while true
        r = rx();

        % ---- Schmidl & Cox sync ----
        [peakIdx, coarseFreq] = schmidl_cox(r, L);
        if isempty(peakIdx), continue; end
        n = (0:length(r)-1).';
        r = r .* exp(-1j*2*pi*coarseFreq*(n)); % coarse CFO corr

        % extract frame after preamble
        frameStart = peakIdx + 2*L + Ncp;
        needLen = (Nfft+Ncp)*numOFDMSym;
        if length(r) < frameStart+needLen, continue; end
        frame = r(frameStart+(0:needLen-1));

        % demodulate
        [rxData, rxPilots] = ofdmDemod(frame);

        % simple channel est. from pilots (average)
        H = mean(rxPilots ./ repmat([1;1;1;-1],[1 numOFDMSym]), 2);
        rxEq = rxData ./ mean(H); % crude eq

        rxBits = qamdemod(rxEq(:), M, 'UnitAveragePower', true);

        % measure BER against random (cannot without same data at RX)
        scatterplot(rxEq(:));
        title('Equalized Constellation');
        pause(0.5);
    end
end

function [peakIdx, freqOff] = schmidl_cox(r, L)
    N = length(r);
    P = zeros(N-L-1,1);
    R = zeros(N-L-1,1);
    for d=1:N-2*L
        P(d) = sum(r(d+1:d+L).*conj(r(d+L+1:d+2*L)));
        R(d) = sum(abs(r(d+L+1:d+2*L)).^2);
    end
    M = abs(P).^2 ./ (R.^2 + eps);
    [~,peakIdx] = max(M);
    freqOff = angle(P(peakIdx))/(2*pi*L);
end
