%dlig=dat with light
%dnon=dat without light
m1=max(dlig);
m2=max(dnon);
s=linspace(0,400,40);
figure;
subplot(2,1,1);
[y1,x]=hist(dlig,s);
hist(dlig,s);
set(gca,'xlim',[-20,400],'fontsize',16);
title('duration in the light chamber');
xlabel('time(s)');
subplot(2,1,2);
[y2,x]=hist(dnon,s);
hist(dnon,s);
set(gca,'xlim',[-20,400],'fontsize',16);
title('duration in nolight chamber');
xlabel('time(s)');

figure;
bar(x,y1,'r');
hold on;
bar(x,y2,'FaceColor',[0.5,0.5,0.5]);
set(gca,'xlim',[-20,400],'fontsize',16);
xlabel('time(s)');
title('distribution of duration in each chamber');

% figure;
% plot(ones(length(dlig),1),dlig,'or');
% hold on;
% plot(2*ones(length(dnon),1),dnon,'og');
% set(gca,'xlim',[0,2.5]);
