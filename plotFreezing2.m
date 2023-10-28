%plot freezing episodes in fear memory
%load data to workspace FIRST
%freezingData=[start,end,duration]
totalSec=trackData(end,1);
fnum=size(freezingData,1);
figure;
%label tones
yyaxis left
%plot(freezingData(:,1),freezingData(:,3),'.','markersize',10);
for i=1:fnum
    plot(freezingData(i,1:2),[1,1]*freezingData(i,3),'-b','linewidth',2);
    hold on;
end
set(gca,'xlim',[0,totalSec],'ylim',[0,30]);
xlabel('Time(sec)');
ylabel('Freezing duration(sec)');
yyaxis right
plot(velocityData(:,1),smooth(velocityData(:,2),15));
ylabel('Moving speed(pixel/sec)');
