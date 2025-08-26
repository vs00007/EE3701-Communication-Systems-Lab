function ts = generatePayload()
    % Generate 100 "Hello world ###" messages
    msg = '';  % initialize as character array
    for k = 0:99
        thisMsg = sprintf('Hello world %03d', k);
        msg = [msg thisMsg];   % concatenate chars
    end
    
    % Convert to ASCII values
    asciiVals = uint8(msg);   % works, since msg is char
    
    % Convert ASCII values to bits (8-bit binary per char)
    bits = de2bi(asciiVals, 8, 'left-msb')';
    bits = bits(:);  % column vector
    
    % Create time vector (sample index)
    t = (0:length(bits)-1)';  
    
    % Wrap in timeseries object
    ts = timeseries(bits, t);
end
