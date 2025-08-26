%% PlutoSDR QAM Link with Sync + BER
% Tested with R2022b+ toolboxes. If an object name differs in your release,
% replace with the closest equivalent.


%% ---------------- User params ----------------
Fc   = 0.5e9;         % RF carrier (Hz)
Fs   = 1.0e6;          % Baseband sample rate (Hz)  (Pluto clk div must support it)
M    = 16;             % QAM order (set 4/16/64 as you like)
SPS  = 4;              % Samples per symbol (RRC oversampling)
roll = 0.35;           % RRC roll-off
span = 10;             % RRC span (symbols)
frmSym = 1000;         % Payload symbols per frame (post-mapper)
barkerLen = 13;        % Barker length used for preamble correlation
txGain = -10;          % dB
rxFrameLen = 40000;    % Pluto receive buffer length (samples)

% Synchronization thresholds
minPreambleCorr = 0.3; % 0..1 (normalized) — tune for your link
maxFramesToTest = 200; % stop after this many receive iterations
plots = true;
% Optional processing
useEqualizer = false;  % Disable equalizer for now (can be complex to train)

rng(42);               % fixed seed for repeatability

%% -------------- System objects ----------------
% SDR
tx = sdrtx('Pluto', ...
    'CenterFrequency', Fc, ...
    'BasebandSampleRate', Fs, ...
    'Gain', txGain);

rx = sdrrx('Pluto', ...
    'CenterFrequency', Fc, ...
    'BasebandSampleRate', Fs, ...
    'SamplesPerFrame', rxFrameLen, ...
    'OutputDataType','double', ...
    'GainSource','AGC Slow Attack');  % Let Pluto do front-end AGC

% RRC filters
txFilt = comm.RaisedCosineTransmitFilter( ...
    'RolloffFactor', roll, ...
    'FilterSpanInSymbols', span, ...
    'OutputSamplesPerSymbol', SPS);

rxFilt = comm.RaisedCosineReceiveFilter( ...
    'RolloffFactor', roll, ...
    'FilterSpanInSymbols', span, ...
    'InputSamplesPerSymbol', SPS, ...
    'DecimationFactor', 1); % keep at SPS; SymbolSynchronizer downsamples

% DSP for sync
cfc = comm.CoarseFrequencyCompensator( ...
    'Modulation','QAM', ...
    'SamplesPerSymbol', SPS, ...
    'FrequencyResolution', 10); % Hz resolution (coarser for faster acquisition)

symSync = comm.SymbolSynchronizer( ...
    'TimingErrorDetector','Gardner (non-data-aided)', ...
    'SamplesPerSymbol', SPS, ...
    'NormalizedLoopBandwidth', 0.02, ...
    'DampingFactor', 1.0);

carSync = comm.CarrierSynchronizer( ...
    'Modulation','QAM', ...
    'SamplesPerSymbol', 1, ...
    'NormalizedLoopBandwidth', 0.02, ...
    'DampingFactor', 1.0);

% DC blocker to mitigate receiver DC spur
dcBlock = dsp.DCBlocker('Algorithm', 'IIR', 'Order', 8);

% Optional equalizer after carrier sync
refConst = qammod((0:M-1).', M, 'gray', 'UnitAveragePower', true);
eql = comm.LinearEqualizer( ...
    'Algorithm','LMS', ...
    'NumTaps', 11, ...
    'ReferenceTap', 6, ...
    'Constellation', refConst, ...
    'StepSize', 1e-3, ...
    'TrainingFlagInputPort', true);

%% -------------- Framing ----------------
% Barker preamble (±1). We'll embed it as QPSK-compatible bits into the QAM stream.
barker = comm.BarkerCode('Length', barkerLen);
bk = step(barker);                 % column vector of ±1 doubles
bkBits = (bk > 0);                 % 1 for +1, 0 for -1
% Map Barker bits into QAM bit lanes: make a QAM preamble of known symbols
% Build enough bits to be a multiple of log2(M)
k = log2(M);
bkBitsPadded = [bkBits; zeros(mod(-numel(bkBits), k), 1)];  %#ok<*AGROW>
bkSym = qammod(bi2de(reshape(bkBitsPadded, k, []).','left-msb'), M, ...
               'gray', 'UnitAveragePower', true);

% Generate one payload frame we will repeat forever from Tx
payloadBits = randi([0 1], frmSym*k, 1);

txIdx = bi2de(reshape(payloadBits, k, []).','left-msb');
payloadSym = qammod(txIdx, M, 'gray', 'UnitAveragePower', true);
% Compose Tx frame = [PreambleSymbols, PayloadSymbols]
txFrameSym = [bkSym; payloadSym];

% Pulse shape
txWave = txFilt(txFrameSym);

% Scale to stay within Pluto DAC limits
txWave = txWave ./ (1.05*max(abs(txWave)));  % soft headroom

% Continuously transmit this one frame
transmitRepeat(tx, txWave);

if plots
    plotConstellation(txFrameSym, M, 'TX symbols (before RRC)');
end

%% -------------- RX processing + BER ----------------
totalBits = 0; bitErrors = 0;
lockedOnce = false;

fprintf('Receiving and estimating BER...\n');
for it = 1:maxFramesToTest
    rxBB = rx();
    if isempty(rxBB), continue; end

    % DC blocking then matched filtering
    rxNoDC = dcBlock(rxBB);
    mf = rxFilt(rxNoDC);

    % Coarse frequency compensation (handles kHz-level offsets)
    cfcOut = cfc(mf);

    % Symbol timing recovery (outputs ~1 sample/symbol)
    symOut = symSync(cfcOut);

    % Carrier/phase sync
    carrOut = carSync(symOut);

    % Optional adaptive equalization (decision-directed LMS)
    if useEqualizer && lockedOnce
        % Only equalize after we have frame lock to avoid training on noise
        % Note: Equalizer training can be complex; disabled by default
        carrOut = eql(carrOut);
    end

    % --------- Frame sync via preamble correlation ----------
    % Build a sliding correlation against known QAM preamble symbols.
    % Use complex correlation; normalize by energy to get 0..1 metric.
    Lp = length(bkSym);
    if length(carrOut) < Lp + frmSym + 20
        continue; % not enough symbols in this chunk
    end

    % Compute normalized correlation quickly using convolution
    % (flip conj for correlation)
    num = abs(conv(carrOut.', flipud(conj(bkSym)), 'valid')).';
    den = sqrt(movsum(abs(carrOut).^2, [Lp-1 0], 'Endpoints','discard') * (bkSym'*conj(bkSym)));
    metric = num ./ max(den, 1e-12);

    [peak, idx] = max(metric);
    if peak < minPreambleCorr
        if plots && mod(it,10)==1
            fprintf('No reliable preamble (peak=%.2f). Continuing...\n', peak);
        end
        continue;
    end

    % Symbol index where payload starts (right after the preamble)
    startSym = idx + Lp;
    if startSym + frmSym - 1 > length(carrOut)
        % wait for more data next loop
        continue;
    end
    lockedOnce = true;

    % Robust phase correction using preamble correlation
    rcvdPre = carrOut(idx : idx + Lp - 1);
    
    % Method 1: LS phase estimate from preamble correlation
    theta1 = angle(rcvdPre' * conj(bkSym));
    
    % Method 2: Average phase offset from individual symbols (more robust)
    if length(rcvdPre) >= 5
        phaseOffsets = zeros(min(5,length(rcvdPre)), 1);
        for i = 1:min(5,length(rcvdPre))
            phaseOffsets(i) = angle(rcvdPre(i) * conj(bkSym(i)));
        end
        % Remove outliers and take median for stability
        phaseOffsets = phaseOffsets(abs(phaseOffsets - median(phaseOffsets)) < 2*std(phaseOffsets));
        theta2 = mean(phaseOffsets);
    else
        theta2 = theta1;
    end
    
    % Use the more stable estimate
    if abs(theta2) < abs(theta1)
        finalTheta = theta2;
    else
        finalTheta = theta1;
    end
    
    % Apply phase correction
    carrOut = carrOut .* exp(-1j*finalTheta);
    
    % Additional fine correction: force alignment with reference constellation
    % This ensures consistent orientation across frames
    refSym = qammod(0, M, 'gray', 'UnitAveragePower', true); % reference symbol at (0,0)
    if abs(rcvdPre(1)) > 0.1  % only if we have a strong preamble symbol
        fineOffset = angle(rcvdPre(1) * conj(refSym));
        carrOut = carrOut .* exp(-1j*fineOffset);
    end

    % Extract payload symbols and demod
    rcvdPayloadSym = carrOut(startSym : startSym + frmSym - 1);
    % --- Demod and BER ---
    rxBits  = qamdemod(rcvdPayloadSym, M, 'gray', 'UnitAveragePower', true, 'OutputType','bit');
    rxBits = rxBits(:);
    
    nBits = numel(payloadBits);                % true bit count for this frame
    [e, ber] = biterr(rxBits, payloadBits);    % biterr returns [errors, BER]
    
    bitErrors = bitErrors + e;
    totalBits = totalBits + nBits;
    
    fprintf('Frame %3d: preamble=%.2f, phase=%.3f rad, BER=%d/%d = %.3g\n', it, peak, finalTheta, e, nBits, ber);

    if plots
        if it==1
            plotConstellation(rcvdPayloadSym, M, 'RX payload (post-sync)');
        end
    end

    % Early exit once we have a solid measurement
    if totalBits >= 5e5 && lockedOnce
        break;
    end
end

if totalBits == 0
    warning('No payload recovered. Try relaxing minPreambleCorr or check RF path.');
else
    fprintf('\n==== Summary ====\n');
    fprintf('Total bits: %d\n', totalBits);
    fprintf('Bit errors: %d\n', bitErrors);
    fprintf('BER       : %.4g\n', bitErrors/totalBits);
end

% Cleanup
release(tx);
release(rx);

%% ---------------- Helper: quick constellation plot ----------------
function plotConstellation(sym, M, ttl)
    figure; plot(real(sym), imag(sym), '.'); axis equal;
    grid on; xlabel('I'); ylabel('Q'); title(sprintf('%s, %d-QAM', ttl, M));
end
