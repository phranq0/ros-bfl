% 1 - (x,y) plot of trajectory
figure,
plot(robPos(:,1),robPos(:,2),'Linewidth',4);
hold on,
scatter(robMeas(:,1),robMeas(:,2),'filled','MarkerFaceColor',[0.976, 0.043, 0.043]);
hold on
plot(robEst(:,1),robEst(:,2),'Color',[0.039, 0.462, 0.074],'Linewidth',4);
grid on,
axis([-1 2 -1 1]);
xlabel("x(m)")
ylabel("y(m)")
legend('Ground Truth','Measurements','Estimate');
title('2D Real and Measured path');

% 2 - Covariance evolution over time
N = length(robMeas);
figure,
plot(1:N,robCov(:,1),'Linewidth',4);
hold on,
plot(1:N,robCov(:,2),'Linewidth',4);
hold on
plot(1:N,robCov(:,3),'Linewidth',4);
grid on,
axis([-1 5 -1 1]);
xlabel("Samples")
ylabel("m")
legend('\sigma^2_x','\sigma^2_y','\sigma^2_\theta');
title('Covariance diagonal evolution');
