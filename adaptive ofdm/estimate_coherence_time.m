% % estimate_coherence_time  Estimate coherence time from channel snapshots (complex-valued supported)
% %   INPUTS:
% %     H_time : Nt x L matrix (rows = time snapshots, cols = taps)
% %     Ts     : (optional) time between snapshots in seconds. If omitted,
% %              outputs are given in frames (lag units).
% %
% %   OUTPUT:
% %     Tc is a struct with fields:
% %       tc_frames_tap_median, tc_frames_tap_mean      -> frames (tap-domain)
% %       tc_frames_subc_median, tc_frames_subc_mean    -> frames (freq-domain)
% %       tc_seconds_*  (only if Ts provided)

% if nargin < 2
%     Ts = [];
% end
% [Nt, L] = size(H_time);

% % PARAMETERS
% thresholds = [0.5, exp(-1)];   % [0.5, 1/e]
% maxlag = Nt-1;

% % Helper: unbiased complex autocorrelation (do NOT take abs here)
% norm_acf = @(x) xcorr(x - mean(x), maxlag, 'biased');
% % We'll later take abs() after normalization

% %% 1) Tap-domain: compute autocorr of each complex tap across time
% acf_taps = zeros(Nt, L);
% for m = 1:L
%     x = H_time(:, m);          % complex time series
%     r = norm_acf(x);           % complex autocorrelation
%     r_pos = r(maxlag+1:end);   % keep nonnegative lags
%     r_pos = r_pos / r_pos(1);  % normalize so r(0)=1 (complex normalization)
%     acf_taps(:, m) = (r_pos);  % use magnitude of normalized ACF
% end

% % Find lag where ACF crosses thresholds (linear interpolation)
% lag_at_thresh_tap = zeros(L, numel(thresholds));
% lags = (0:maxlag)'; % column vector
% for m = 1:L
%     r = abs(acf_taps(:, m));
%     for t = 1:numel(thresholds)
%         thr = thresholds(t);
%         idx = find(r <= thr, 1, 'first');
%         if isempty(idx)
%             lag_at_thresh_tap(m,t) = nan;
%         elseif idx == 1
%             lag_at_thresh_tap(m,t) = 0;
%         else
%             % linear interpolation for sub-frame precision
%             x1 = r(idx-1); x2 = r(idx);
%             lag_interp = (idx-1) + (x1 - thr)/(x1 - x2);
%             lag_at_thresh_tap(m,t) = lag_interp - 1;
%         end
%     end
% end

% %% 2) Frequency-domain: compute per-subcarrier autocorr
% H_freq = fft(H_time, [], 2); % Nt x L
% acf_subc = zeros(Nt, L);
% for k = 1:L
%     x = H_freq(:, k);          % complex subcarrier series
%     r = norm_acf(x);
%     r_pos = r(maxlag+1:end);
%     r_pos = r_pos / r_pos(1);
%     acf_subc(:, k) = (r_pos);
% end

% lag_at_thresh_subc = zeros(L, numel(thresholds));
% for k = 1:L
%     r = acf_subc(:, k);
%     for t = 1:numel(thresholds)
%         thr = thresholds(t);
%         idx = find(r <= thr, 1, 'first');
%         if isempty(idx)
%             lag_at_thresh_subc(k,t) = nan;
%         elseif idx == 1
%             lag_at_thresh_subc(k,t) = 0;
%         else
%             x1 = r(idx-1); x2 = r(idx);
%             lag_interp = (idx-1) + (x1 - thr)/(x1 - x2);
%             lag_at_thresh_subc(k,t) = lag_interp - 1;
%         end
%     end
% end

% %% Summarize
% Tc.tc_frames_tap_median     = median(lag_at_thresh_tap(:,1),'omitnan');
% Tc.tc_frames_tap_mean       = mean(lag_at_thresh_tap(:,1),'omitnan');
% Tc.tc_frames_tap_median_1e  = median(lag_at_thresh_tap(:,2),'omitnan');
% Tc.tc_frames_tap_mean_1e    = mean(lag_at_thresh_tap(:,2),'omitnan');

% Tc.tc_frames_subc_median    = median(lag_at_thresh_subc(:,1),'omitnan');
% Tc.tc_frames_subc_mean      = mean(lag_at_thresh_subc(:,1),'omitnan');
% Tc.tc_frames_subc_median_1e = median(lag_at_thresh_subc(:,2),'omitnan');
% Tc.tc_frames_subc_mean_1e   = mean(lag_at_thresh_subc(:,2),'omitnan');

% % Store raw data
% Tc.lag_at_thresh_tap = lag_at_thresh_tap;
% Tc.lag_at_thresh_subc = lag_at_thresh_subc;
% Tc.acf_taps = acf_taps;
% Tc.acf_subc = acf_subc;
% Tc.lags = lags;

% % Convert to seconds if Ts provided
% if ~isempty(Ts)
%     Tc.tc_seconds_tap_median      = Tc.tc_frames_tap_median * Ts;
%     Tc.tc_seconds_tap_mean        = Tc.tc_frames_tap_mean * Ts;
%     Tc.tc_seconds_subc_median     = Tc.tc_frames_subc_median * Ts;
%     Tc.tc_seconds_subc_mean       = Tc.tc_frames_subc_mean * Ts;
%     Tc.tc_seconds_tap_median_1e   = Tc.tc_frames_tap_median_1e * Ts;
%     Tc.tc_seconds_subc_median_1e  = Tc.tc_frames_subc_median_1e * Ts;
% else
%     Tc.tc_seconds_tap_median = nan;
% end

% end

function res = estimate_coherence_time(H, Ts, varargin)
% ESTIMATE_COHERENCE_TIME  Estimate channel coherence time from taps
%
% res = estimate_coherence_time(H, Ts) where
%   H    : T x K complex matrix (T samples, K taps; your case K=4)
%   Ts   : sampling interval in seconds (time between successive rows)
%
% Optional name-value arguments:
%   'MaxLag'   : maximum lag in samples (default floor(T/4))
%   'Criterion': '1/e' (default), '0.5', or numeric threshold in (0,1)
%   'Method'    : 'per-tap' (default) or 'vector' (treat each row as vector
%                 and compute vector autocorr via inner product)
%   'Plot'      : true/false (default true) display rho vs time and PSD
%
% Returned struct res contains:
%   res.lags, res.times, res.rho_abs, res.Tc_sample (lag index), res.Tc (seconds)
%   res.Tc_interp (interpolated seconds), res.PSD, res.freq, res.Tc_from_Doppler
%   res.rho_per_tap (K x maxLag)
%
% Example:
%   res = estimate_coherence_time(H, 0.01, 'MaxLag',200, 'Criterion','1/e');

%% parse inputs
p = inputParser;
addRequired(p,'H',@(x) ismatrix(x) && ~isempty(x));
addRequired(p,'Ts',@(x) isscalar(x) && x>0);
addParameter(p,'MaxLag',[],@(x) isempty(x) || (isscalar(x) && x>0));
addParameter(p,'Criterion',0.1,@(x) ischar(x) || isnumeric(x));
addParameter(p,'Method','vector',@(x) any(validatestring(x,{'per-tap','vector'})));
addParameter(p,'Plot',true,@islogical);
parse(p,H,Ts,varargin{:});
MaxLag = p.Results.MaxLag;
criterion = p.Results.Criterion;
method = p.Results.Method;
doPlot = p.Results.Plot;

[T, K] = size(H);
if isempty(MaxLag)
    MaxLag = floor(T/4);
else
    MaxLag = min(MaxLag, floor(T-1));
end

% prepare threshold
if ischar(criterion)
    if strcmp(criterion,'1/e'); thresh = 1/exp(1);
    elseif strcmp(criterion,'0.5'); thresh = 0.5;
    else error('Unknown string criterion'); end
else
    thresh = double(criterion);
end

%% compute normalized autocorrelations
lags = (0:MaxLag).';
times = lags * Ts;

if strcmp(method,'per-tap')
    rho_per_tap = zeros(K, MaxLag+1);
    for k=1:K
        h = H(:,k);
        r0 = mean(h .* conj(h));          % R(0)
        for d=0:MaxLag
            if d==0
                r = r0;
            else
                r = mean(h(1:T-d) .* conj(h(1+d:T)));
            end
            rho_per_tap(k, d+1) = r ./ (r0 + eps);
        end
    end
    rho = mean(rho_per_tap,1);   % average (complex) across taps
else
    % vector method: use inner product of tap vectors as single complex series
    % c(t) = <H(t,:), H(t+tau,:)> and normalize by tau=0
    r0_vec = mean(sum(H .* conj(H),2)); % scalar
    rho_vec = zeros(1, MaxLag+1);
    for d=0:MaxLag
        if d==0
            r = r0_vec;
        else
            % inner product averaged over t
            r = mean( sum( H(1:T-d,:) .* conj(H(1+d:T,:)), 2 ) );
        end
        rho_vec(d+1) = r ./ (r0_vec + eps);
    end
    rho_per_tap = [];  % not computed in this mode
    rho = rho_vec;
end

rho_abs = abs(rho);

%% find coherence time (first crossing <= thresh)
idx_below = find(rho_abs <= thresh, 1, 'first');
if isempty(idx_below)
    Tc_sample = NaN;
    Tc = NaN;
    Tc_interp = NaN;
else
    Tc_sample = idx_below - 1;  % lag index (0-based)
    Tc = times(idx_below);      % coarse (sample-aligned)
    % linear interpolation between previous and current lag for better estimate:
    if idx_below > 1
        y1 = rho_abs(idx_below-1);
        y2 = rho_abs(idx_below);
        x1 = times(idx_below-1);
        x2 = times(idx_below);
        if y1==y2
            Tc_interp = Tc;
        else
            % find t where linear interp hits thresh
            Tc_interp = x1 + (thresh - y1) * (x2 - x1) / (y2 - y1);
        end
    else
        Tc_interp = Tc;
    end
end

%% Doppler estimate via PSD of average autocorrelation (Wiener-Khinchin)
% Build averaged autocorrelation (one-sided symmetric) -> compute PSD
% Use rho (complex) as lags 0..MaxLag. Form symmetric sequence:
R_pos = rho;  % length MaxLag+1
% form full autocorrelation with negative lags by conj symmetry
R_full = [conj(R_pos(end:-1:2)), R_pos]; % length 2*MaxLag+1
Nfft = 2^nextpow2(length(R_full)*4);  % zero-pad for frequency resolution
S = fftshift(fft(R_full, Nfft));
fs = 1/Ts;
freq = linspace(-fs/2, fs/2, Nfft);
PSD = abs(S);  % magnitude of PSD (proportional to Doppler PSD)

% normalize PSD to unit area for moment calculation
PSD_norm = PSD ./ (sum(PSD) + eps);

% RMS Doppler (frequency) using spectral moments (two-sided)
f_abs = abs(freq);
f_rms = sqrt( sum( (f_abs.^2) .* PSD_norm ) );

% convert to time estimate: Tc ≈ 1/(2*pi * f_rms)
Tc_from_Doppler = NaN;
if f_rms > 0
    Tc_from_Doppler = 1 / (2*pi * f_rms);
end

%% Collect results
res.H = H;
res.Ts = Ts;
res.T = T;
res.K = K;
res.lags = lags;
res.times = times;
res.rho = rho;
res.rho_abs = rho_abs;
res.rho_per_tap = rho_per_tap;
res.Tc_sample = Tc_sample;
res.Tc = Tc;
res.Tc_interp = Tc_interp;
res.criterion = criterion;
res.PSD = PSD;
res.freq = freq;
res.PSD_norm = PSD_norm;
res.f_rms = f_rms;
res.Tc_from_Doppler = Tc_from_Doppler;

%% Plot
if doPlot
    figure('Name','Coherence estimation','NumberTitle','off','Units','normalized','Position',[0.1 0.2 0.6 0.6]);
    subplot(2,1,1)
    plot(times, rho_abs, 'LineWidth', 1.5)
    hold on
    yline(thresh, '--k','Threshold','LabelHorizontalAlignment','left');
    if ~isnan(Tc), xline(Tc, ':r', sprintf('Tc=%.4g s',Tc)); end
    if ~isnan(Tc_interp)
        xline(Tc_interp, '-.m', sprintf('Tc_{interp}=%.4g s', Tc_interp));
    end
    xlabel('Time (s)')
    ylabel('|\\rho(\\tau)|')
    title('Average normalized autocorrelation vs time')
    grid on
    hold off

    subplot(2,1,2)
    % plot PSD vs frequency (Hz)
    plot(freq, PSD_norm, 'LineWidth', 1.2)
    xlabel('Frequency (Hz)')
    ylabel('Normalized PSD')
    title('Doppler PSD (from autocorrelation via FFT)')
    xlim([-fs/2 fs/2])
    grid on
    sgtitle(sprintf('Estimated Tc: %.4g s (sample), %.4g s (interp), Doppler-Tc ≈ %.4g s', ...
                    res.Tc, res.Tc_interp, res.Tc_from_Doppler))
end

end
