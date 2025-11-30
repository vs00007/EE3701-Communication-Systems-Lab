function trollface_img = decode_Trollface(received_data)
% decode_Trollface - reconstructs the 48x48 pixel trollface from received BPSK data
%
%   Input:
%       received_data : 1 x 2304 vector (real or complex BPSK samples)
%
%   Output:
%       trollface_img : 48x48 logical matrix (1 = black, 0 = white)

    % Ensure it's a column vector
    received_data = received_data(:);

    % Check length
    if length(received_data) ~= 2304
        error('Expected 2304 samples, got %d', length(received_data));
    end

    %% Step 1: Demodulate BPSK → bits
    % Decision threshold at 0
    bits = real(received_data) > 0;  % 1 for +1, 0 for -1

    %% Step 2: Reshape into 48x48 matrix
    % We originally transmitted row-major order ('.'' transpose)
    trollface_img = reshape(bits, 48, 48).'; % transpose back
    % 
    % %% Step 3: Display
    % figure;
    % imagesc(~trollface_img);      % invert for white background
    % colormap(gray(2));
    % axis equal off;
    % title('Decoded Trollface Image');

end