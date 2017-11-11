filename = 'motor_sim.dat'

data = csvread(filename);


t = data(:,1);
dt = data(2,1) - data(1,1);
X = data(:,2:8);
Xd = data(:,9:15);
M = data(:,16:19);

figure(1)
hold on


% V_l
subplot(2,2,1)
plot(t,M(:,2),'-');
xlabel('t')
ylabel('V');
title('Input voltage to left motor')

% W_l
subplot(2,2,2)
plot(t,X(:,7),'-');
xlabel('t')
ylabel('\omega_l')
title('Velocity response')

% V_R
subplot(2,2,3)
plot(t,M(:,1),'-');
xlabel('t')
ylabel('V')
title('Input voltage to right motor')

%W_R
subplot(2,2,4)
plot(t,X(:,6),'-');
xlabel('t')
ylabel('\omega_r')
title('Velocity response')



figure()
hold on;

plot(X(:,1),X(:,2))