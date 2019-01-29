%% Reading Data
s = file2wave('../outputs/usrp_samples_default_fpga_HG_image-0.dat');
figure; hold on; plot(real(s)); plot(imag(s)); title('issue 4: impulse');
s = file2wave('../outputs/usrp_samples_default_fpga_HG_image_spectrumtest_34p5db.dat');
% figure; hold on; plot(real(s)); plot(imag(s));
figure; obw(s,125e6); title('OBW: TX gain = 34.5');
s = file2wave('../outputs/usrp_samples_default_fpga_HG_image_spectrumtest_35db.dat');
% figure; hold on; plot(real(s)); plot(imag(s));
figure; obw(s,125e6); title('OBW: TX gain = 35');

%% generating waveforms
N = 2048 - 256-512;
fs = 125e6;
n_zeroapp = 512;

fbw = 100e6;

t1 = ([0:(N-1)] - N/2).*(1/fs);

wfrm = exp(1i*pi*.5*(fbw/(t1(end)))*t1.^2);
wfrm = [zeros(1,n_zeroapp),wfrm];
wave2file(wfrm,'../waveforms/waveform_data.bin');

scale = .5*double(intmax('int16'));
wave2file(real(wfrm),imag(wfrm),'../waveforms/waveform_data_halfgain.bin',numel(wfrm),'int16',scale);
% wave2file(real(wfrm),imag(wfrm),'../waveforms/chirpN100_halfgain.bin',numel(wfrm),'int16',scale);
