function out = gust_cos_input( amplitude, fraction_time)
%GUST 1-COSINE
%   Calcola l'input per una raffica (andamento 1-cos)
%   (Valori iniziali amplitude = 0, fraction_time=0.05)

% Time
dt=0.015;       % sampling time
tmin=0;         % start time
tmax=5;        % end time
t=[tmin:dt:tmax]'; % Column vector

% "1 - Cosine" gust input 
gust_amp_1_minus_cos = amplitude;       % max velocity of "1 - cosine" gust   (m/s)
gust_t = fraction_time;   %  fraction of total data time length that is gust    0 - 1 

% GUST INPUT TERMS - "1-cosine"
  Sgust = zeros(size(t));
  %  1 - Cosine gust
  g_end = tmax * gust_t;
  gt = sum(t < g_end);  % ultimo indice di t dove arriva la raffica
if gust_amp_1_minus_cos ~= 0
  for ii = 1:gt
    Sgust(ii) = gust_amp_1_minus_cos/2 * (1 - cos(2*pi*t(ii)/g_end));
  end
end

out = struct('t', t, 'u', Sgust');

end