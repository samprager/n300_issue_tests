function data_iq = wave2file(varargin)
% wave2file - generates fixed point waveform file compatible with MiXIL usrp radar
%
% Syntax:  (output optional) data_iq =
%                function_name(complex data,filename)
%                function_name(real data, imag data,filename)  
%                function_name(real data, imag data,filename,length)  
%                function_name(real data, imag data,filename,length,format)  
%                function_name(real data, imag data,filename,length,format, scale)
%
% Inputs:
%    complex data - complex waveform with scale ranging from +- 1
%    real data, imag data - real, imag waveform with scale ranging from +- 1
%    filename - output filename (.bin extension recommended)
%    length - length to write to file (used to zero pad waveform)
%    format - fixed point format to write data with (default 'int16') 
%    scale - defines range of waveform values (default intmax(format))
%
% Outputs:
%    dataiq - (optional) fixed point complex data written to file
%

% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: file2wave()

% Author: Samuel Prager
% University of Southern California
% email: sprager@usc.edu
% Created: 2016/05/12 01:00:00; Last Revised: 2017/03/12 19:56:56

%------------- BEGIN CODE --------------
if (nargin ==2)
    I = real(varargin{1});
    Q = imag(varargin{1});
    fname = varargin{2};
    n = numel(I);
    format = 'int16';
    scale = double(intmax(format));
elseif (nargin ==3)
    I = varargin{1};
    Q = varargin{2};
    fname = varargin{3};
    n = numel(I);
    format = 'int16';
    scale = double(intmax(format));
elseif (nargin ==4)
    I = varargin{1};
    Q = varargin{2};
    fname = varargin{3};
    n = varargin{4};
    format = 'int16';
    scale = double(intmax(format));
elseif (nargin ==5)
    I = varargin{1};
    Q = varargin{2};
    fname = varargin{3};
    n = varargin{4};
    format = varargin{5};
    scale = double(intmax(format));
elseif (nargin ==6)
    I = varargin{1};
    Q = varargin{2};
    fname = varargin{3};
    n = varargin{4};
    format = varargin{5};
    scale = varargin{6};
else    
    error('wave2file(): Unrecognized argument list. Usage: wave2file(I,Q,fname,(length),(format),(scale))');
end

maxval = max(abs([I(:);Q(:)]));

I = [I(:)',zeros(1,n-numel(I))];
Q = [Q(:)',zeros(1,n-numel(Q))];

formatfunc = str2func(format);
data_i = formatfunc((scale/maxval)*I);
data_q = formatfunc((scale/maxval)*Q);

data_iq = reshape([data_q;data_i],1,2*n);
fileID = fopen(fname,'w');
frewind(fileID);
fwrite(fileID,data_iq,format);
fclose(fileID);

end
%------------- END OF CODE --------------