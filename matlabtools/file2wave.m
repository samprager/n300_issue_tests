function varargout = file2wave(varargin)

if (nargin ==1)
    fname = varargin{1};
    format = 'int16';
    scale_out = double(intmax(format));
elseif (nargin ==2)
    fname = varargin{1};
    format = varargin{2};
    scale_out = double(intmax(format));
elseif (nargin ==3)
    fname = varargin{1};
    format = varargin{2};
    scale_out = varargin{3};
else   
    error('file2wave(): Unrecognized argument list. Usage: file2wave(fname,(format),(scale))');
end

fileID = fopen(fname,'r');
data=fread(fileID,format);
fclose(fileID);

[d, base, ext] = fileparts(fname);

if (strcmp(ext,'.dat')|| strcmp(ext,'.ref'))
    I = data(1:2:end)';
    Q = data(2:2:end)';
else
    I = data(2:2:end)';
    Q = data(1:2:end)';
end

scale = scale_out/double(intmax(format));

I = scale*I;
Q = scale*Q;

if (nargout == 0)
    hold on; plot(I); plot(Q); legend('I','Q'); hold off;
    return;
elseif (nargout == 1)
    varargout{1} = I+1i*Q;
    return;
elseif (nargout == 2)
    varargout{1} = I;
    varargout{2} = Q;
    return;
else 
    error('file2wave(): Unrecognized output type. Output formats: [],[IQ],[I,Q]');
end