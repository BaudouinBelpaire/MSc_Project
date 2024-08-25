v_max = 0.5;
a_max = 0.5;
x_f = 2;

t1f = 2*v_max/a_max;

t1 = 0:0.1:t1f;
a = a_max/t1f;
a_1 = a * t1
v_1 = a/2 * t1.^2
x_1 = a/6 .* t1.^3

delta_t3 = 2*v_1(end)/a_max
x2f = x_f + a_max/2*(delta_t3).^2 - a_max/(6*delta_t3)*delta_t3.^3 - v_1(end)*delta_t3

t2f = (x2f - x_1(end))/v_1(end) + t1f
t2 = t1f+0.1:0.1:t2f-0.1
a_2 = zeros(size(t2))
v_2 = v_1(end) .* ones(size(t2));
x_2 = x_1(end) + v_2 .* (t2 - t1f)

t3f = t2f + delta_t3
t3 = t2f:0.1:t3f
a_3 = -a_max + a_max/(t3f-t2f)*(t3-t2f)
v_3 = -a_max*(t3-t2f) + a_max*(t3-t2f).^2/(2*(t3f-t2f)) + v_2(end)
x_3 = v_2(end)*(t3-t2f) + -a_max*(t3-t2f).^2/2 + a_max*(t3-t2f).^3/(6*(t3f-t2f)) + x2f 

% Combine all the phases into one
t_total = [t1 t2 t3];
a_total = [a_1 a_2 a_3];
v_total = [v_1 v_2 v_3];
x_total = [x_1 x_2 x_3];


% Plot the results
figure;

% Plot acceleration
subplot(3,1,1);
plot(t_total, a_total, 'b-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Acceleration Profile');
grid on;

% Plot velocity
subplot(3,1,2);
plot(t_total, v_total, 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity Profile');
grid on;

% Plot position
subplot(3,1,3);
plot(t_total, x_total, 'g-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position (m)');
title('Position Profile');
grid on;
