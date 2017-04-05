%% Bitcraze experiment used pwm (0-256) as the input and measured force

pwm_bc = 1:256;

a_bc = 0.409e-3;
b_bc = 140.5e-3;
c_bc = -0.999;

F_bc = 0.409e-3 * pwm_bc.^2 + 140.5e-3 * pwm_bc - 0.999;

%% Non dimensionalize the pwm input to (0-1)
a_nd = a_bc * 256^2;
b_nd = b_bc * 256;
c_nd = c_bc;
pwm_nd = 0:.01:1;

F_nd = a_nd * pwm_nd.^2 + b_nd * pwm_nd + c_nd;

%% Use quadratic formula to invert the non dimensional result
% The goal here is to calculate the pwm input to create a desired thrust

% Hard code these as parameters for the crazyflies
c1 = -b_nd / (2*a_nd);
c2 = 1/sqrt(a_nd);
c3 = (b_nd^2/(4*a_nd)) - c_nd;

thrust=0:70;
power_in = c1 + c2 * sqrt(c3 + thrust);

hold on;
axis([0 70 0 1.1]);
plot(F_bc, (pwm_bc/256), 'b');
plot(thrust, power_in, 'mo');
