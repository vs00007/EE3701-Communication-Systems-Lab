function txOFDM
    fs = 1e6; centerFreq = 0.5e9;
    Nfft = 64; Ncp = 16; M = 4; numOFDMSym = 20;
    pilotIdx = [12;26;40;54];

    % OFDM modulator
    ofdmMod = comm.OFDMModulator( ...
        'FFTLength', Nfft, ...
        'NumGuardBandCarriers', [6;5], ...
        'InsertDCNull', true, ...
        'PilotInputPort', true, ...
        'PilotCarrierIndices', pilotIdx, ...
        'CyclicPrefixLength', Ncp, ...
        'NumSymbols', numOFDMSym);

    infoMod = info(ofdmMod);
    dataSize = infoMod.DataInputSize;

    % Preamble (Schmidl & Cox: two identical halves)
    L = Nfft/2;
    X = [randi([0 1], L, 1)*2-1; zeros(L,1)]; % BPSK in half-band
    td = ifft(ifftshift(X));
    preamble = [td(end-Ncp+1:end); td; td]; % CP + two halves

    % Pluto TX
    tx = sdrtx('Pluto', 'CenterFrequency', centerFreq, ...
        'BasebandSampleRate', fs, 'Gain', -10);

    fprintf("TX ready. Streaming...\n");

    while true
        % random QPSK data
        data = randi([0 M-1], dataSize);
        symb = qammod(data(:), M, 'UnitAveragePower', true);
        symb = reshape(symb, dataSize);

        % fixed pilots (BPSK)
        pilotSym = repmat([1;1;1;-1], [1 numOFDMSym]);

        % OFDM time waveform
        txWave = ofdmMod(symb, pilotSym);

        % Frame = preamble + OFDM
        txFrame = [preamble; txWave];
        txFrame = txFrame / max(abs(txFrame)) * 0.8; % scale

        tx(txFrame);
        pause(0.05);
    end
end
