function line_tracker_min_jerk_numerical_issues()
% line_track_min_jerk_numerical_issues
%
% Written by Justin Thomas, 5/6/2016

total_t = 10;

% Boundary conditions
bcs = [
    0;    % xo
    100;  % xf
    10;   % vo
    0;    % vf
    0;    % ao
    0];   % af

x_old = current_method(total_t, bcs);
x_new = new_method(total_t, bcs);

%% Evaluate

tvec = 0:total_t/100:total_t;

eval_old = evaluate_poly(x_old, tvec);

eval_new = evaluate_poly(x_new, tvec / total_t);

% Scale position (not needed)
eval_new(1,:) = eval_new(1,:);
% Scale velocity
eval_new(2,:) = 1 / total_t * eval_new(2,:);
% Scale acceleration
eval_new(3,:) = 1 / total_t^2 * eval_new(3,:);

close
figure;
hold all;
for idx = 1:3
    plot(tvec, eval_old(idx,:), '.');
    plot(tvec, eval_new(idx,:), 'o');
end

end

function [x] = current_method(total_t, bcs)

dt = total_t;
dt2 = dt^2;
dt3 = dt^3;
dt4 = dt^4;
dt5 = dt^5;

% syms dt dt2 dt3 dt4 dt5 real;

A = [ 1,  0,      0,       0,        0,        0;
      1, dt,    dt2,     dt3,      dt4,      dt5;
      0,  1,      0,       0,        0,        0;
      0,  1, 2 * dt, 3 * dt2,  4 * dt3,  5 * dt4;
      0,  0,      2,       0,        0,        0;
      0,  0,      2,  6 * dt, 12 * dt2, 20 * dt3];
  
disp(['Old method, condition number:', num2str(cond(A))]);

x = A \ bcs;

end

function x = new_method(total_t, bcs)

% Assume dt = 1
A = [ 1,  0,  0,  0,  0,  0;
      1,  1,  1,  1,  1,  1;
      0,  1,  0,  0,  0,  0;
      0,  1,  2,  3,  4,  5;
      0,  0,  2,  0,  0,  0;
      0,  0,  2,  6, 12, 20];

% To get a non-numerical solution to inv(A)
% syms a b c d e f g h real;
% A = [ a,  0,  0,  0,  0,  0;
%       a,  a,  a,  a,  a,  a;
%       0,  a,  0,  0,  0,  0;
%       0,  a,  b,  c,  d,  e;
%       0,  0,  b,  0,  0,  0;
%       0,  0,  b,  f,  g,  h];
% Ainv = subs(simplify(inv(A)), {'a','b','c','d','e','f','g','h'}, {1,2,3,4,5,6,12,20})

Ainv = [...
    1,   0,  0,  0,    0,   0;
    0,   0,  1,  0,    0,   0;
    0,   0,  0,  0,  1/2,   0;
  -10,  10, -6, -4, -3/2, 1/2;
   15, -15,  8,  7,  3/2,  -1;
   -6,   6, -3, -3, -1/2, 1/2];

% A_yaw = [ 1,  0,  0,  0;
%           1,  1,  1,  1;
%           0,  1,  0,  0;
%           0,  1,  2,  3];
% 
% A_yaw_inv = inv(A_yaw);
  
%% Scale derivatives so the boundary conditions are correct despite the time scaling

% We can scale the rows of A or we can scale the boundary conditions. The nice
% thing about scaling boundary conditions is that we can precompute inv(A).

% A([3,4],:) = 1 / total_t   * A([3,4],:);  % Velocities
% 
% A([5,6],:) = 1 / total_t^2 * A([5,6],:);  % Accelerations

disp(['New method, condition number:', num2str(cond(A))]);

bcs([3,4]) = total_t   * bcs([3,4]);        % Velocities
bcs([5,6]) = total_t^2 * bcs([5,6]);        % Accelerations

x = Ainv * bcs;

end

function b = evaluate_poly(coeffs, t)

n = length(t);

% Make sure t is in the 3rd dimension
t = reshape(t(:),[1,1,n]);

l = ones(1,1,n);
o = zeros(1,1,n);

A = [...
     l, t, t.^2,   t.^3,    t.^4,    t.^5;
     o, l,  2*t, 3*t.^2,  4*t.^3,  5*t.^4;
     o, o,  2*l,    6*t, 12*t.^2, 20*t.^3;
     o, o,    o,    6*l,    24*t, 60*t.^2];
 
b = zeros(size(A,1),n);
for idx = 1:n
    b(:,idx) = A(:,:,idx) * coeffs;
end

end