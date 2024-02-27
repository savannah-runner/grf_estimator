path = '/home/savanna-runner/log_data/grf_RR.csv';

output = dlmread(path);
dt = 0.0005;
Time = [1:length(output)]*dt;

figure(1); 
subplot(2,1,1); hold on;
plot(Time, output(:,3),'r','LineWidth',6);

figure_duration = [0 3];

fs = 2000;
f_cut = 100; % cut off frequency in freq. domain.

%w = (fs/length(Time))*(1:length(Time));
cut_index = round(f_cut/fs*numel(Time));
O = zeros(3,length(Time));
output_lpf = zeros(3,length(Time));
for i = 1:3
    O(i,:) = fft(output(:,i));
    O(i,cut_index+2:end-cut_index) = 0;
    output_lpf(i,:) = ifft(O(i,:));
end

figure(1);
subplot(2,1,1); hold on;
plot(Time, output_lpf(3,:),'k','LineWidth',3);
plot(Time, 0*Time,'b','LineWidth',3);
ylim([-300 1000]);
xlim(figure_duration);
xlabel('Time[sec]');
ylabel('Ground Reaction Force[N]');

Fx = output_lpf(1,:);
Fy = output_lpf(2,:);
Fz = output_lpf(3,:);
Ft = sqrt(Fx.^2+Fy.^2);

friction_ratio = Ft./abs(Fz);

threshold_grf = 300;
indicator = 0;

contact_phase = zeros(1,length(Time));
for i = 1:length(Time)
    if Fz(i)>threshold_grf
        indicator = 1;
    end

    if Fz(i)<0 && indicator == 1
        indicator = 0;
    end

    if indicator == 1
        contact_phase(i) = 1;
    end
    
end

figure(1);
subplot(2,1,1);
plot(Time,500*contact_phase,'g','LineWidth',3);
plot(Time,0*Time,'b','LineWidth',3);


for i = 1:length(Time)
    if contact_phase(i) == 0
        friction_ratio(i) = 5;
    end
end

figure(1);
subplot(2,1,2); hold on;
bar(Time, friction_ratio,1,'k');
xlim(figure_duration);
ylim([1 1.01]);
