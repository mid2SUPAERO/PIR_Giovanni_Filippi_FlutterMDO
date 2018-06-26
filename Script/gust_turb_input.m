function out = gust_turb_input( amplitude, fraction_time)

%(Valori iniziali amplitude=10, fraction_time=0.7)
% TURBULENCE INPUT - random signal between 0 Hz and turb_max_freq Hz
%   uniform amplitude across random frequency input
turb_amp = amplitude;       % max vertical velocity of turbulence   (m/s)
turb_t = fraction_time;   %  fraction of total time length that is turbulence    0 - 1 
turb_max_freq =20;    % max frequency of the turbulence (Hz)

% Time
dt=0.01;   % sampling time
tmin=0;     % start time
tmax=5;    % end time
t=[tmin:dt:tmax]';       % Column vector

% TURBULENCE INPUT - unform random amplitude between 0 Hz and turb_max_freq Hz
  Sturb = zeros(size(t));
  t_end = tmax * turb_t;
  tpts = sum(t < t_end);  % number of turbulence time points required
  npts = max(size(t));
  if rem(max(tpts),2) ~= 0
    tpts =  tpts- 1;
  end
if turb_amp ~= 0
  td2 = tpts / 2;
  td2p1 = tpts/2 + 1;
  df = 1/(tpts*dt);
  fpts = fix(turb_max_freq / df) + 1; %  number of frequency points that are required to form turbulence series

  for ii = 1:fpts              % define real and imag parts of freq domain terms magnitude of unity, random phase
    a(ii) = 2 * rand - 1;    % real part  - 1 < a < 1
    b(ii) = sqrt(1 - a(ii)*a(ii)) * (2*round(rand) - 1);   % imag part  
  end
    
  tf =  (a + j*b);   %  determine complex frequency representation with correct frequency characteristics
  tf(fpts+1 : td2p1) = 0;
  tf(td2p1+1 : tpts) = conj(tf(td2:-1:2));
  Sturb(1:tpts) = turb_amp * real(ifft(tf));
  kscale = turb_amp / max(Sturb);
  Sturb = kscale*Sturb;
else
    Sturb(1:tpts)=zeros(tpts,1);
end

out = struct('t', t, 'u', Sturb');

end