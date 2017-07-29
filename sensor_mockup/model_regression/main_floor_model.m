%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
% Date: 2017-07-28
% License: GPLv3
% Matlab: R2016b
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

values = readtable('floor_prox_sense_amiro38.csv');

% Colate the values
x = repmat(values.colorvalue,[4,1]);
y = [values.FLOOR_PROX_FRONT_LEFT; ...
     values.FLOOR_PROX_FRONT_RIGHT; ...
     values.FLOOR_PROX_WHEEL_LEFT; ...
     values.FLOOR_PROX_WHEEL_RIGHT];
 
% Get the specific variance for every value
valuesUnique = unique(x);
y_sigma = zeros(size(valuesUnique));
for idx = 1 : numel(y_sigma)
    y_sigma(idx) = std(y(x == valuesUnique(idx)));
end

subplot(2,1,1)
plot(x,y,'o')
title('Sensor Values over Grayscale')
ylim([0 2^16])
xlim([0 256])
grid on
subplot(2,1,2)
stem(valuesUnique,y_sigma)
title('Standard Deviation of Sensor Values over Grayscale')
xlim([0 256])
grid on

[fitresult, gof] = func_floor_model_fit(x, y, valuesUnique, y_sigma);

return
%% Models
% Value fitting
% a*atan(b*x+d)+c
% fitresult{2}.a = 11010.7641914599
% fitresult{2}.b = 0.0610333423539444
% fitresult{2}.c = 21783.9217626851
% fitresult{2}.d = -7.94411704377273

% Deviation fitting
% a*tanh(b*x+d)+c
% fitresult{4}.a = 1108.83338439758
% fitresult{4}.b = 0.0223713977792142
% fitresult{4}.c = 1503.24485506827
% fitresult{4}.d = -2.08504831316051

% % Open the toolbox
sftool('floor_model.sfit')