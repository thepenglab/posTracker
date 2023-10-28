%calculate the residual times in each chamber
%function resid(filename)
%load(filename);
tkDat=round(trackData(:,2:3));
ptnum=length(trackData);
locTag=zeros(ptnum,1);
for i=1:ptnum
    locTag(i)=chamberMap(tkDat(i,2),tkDat(i,1));
end
d=locTag(2:end)-locTag(1:end-1);
%for each chamber
chamberNum=2;
tData=cell(1,chamberNum);
epMean=zeros(chamberNum,3);      %[mean,sem,n]
tmx=0;
for i=1:chamberNum
    %timepoint to enter and exit chamber1
    c1=find(d~=0 & locTag(2:end)==i);
    c2=find(d~=0 & locTag(1:end-1)==i);
    n=max(length(c1),length(c2));
    dat=zeros(n,3);     %[enter,exit,duration]
    if length(c1)>length(c2)
        dat(:,1)=trackData(c1+1,1);
        dat(:,2)=[trackData(c2+1,1);trackData(end,1)];
    elseif length(c1)==length(c2)
        if c2(1)>c1(1)
            dat(:,1)=trackData(c1+1,1);
            dat(:,2)=trackData(c2+1,1);
        else
            dat(:,1)=trackData(c1+1,1);
            dat(:,2)=[trackData(c2(2:end)+1,1);trackData(end,1)];
        end
    else
        dat(:,1)=[trackData(1,1);trackData(c1+1,1)];
        dat(:,2)=trackData(c2+1,1);
    end
    dat(:,3)=dat(:,2)-dat(:,1);
    %consider border issues
    idx=find(dat(2:end,1)-dat(1:end-1,2)<1);
    if ~isempty(idx)
        dat(idx,2)=dat(idx+1,2);
        dat(idx,3)=dat(idx,2)-dat(idx,1);
        idx2=setdiff(1:n,idx+1);
        dat=dat(idx2,:);
    end
    
    

    %ignore the rest of short-enter
    idx=find(dat(:,3)>1);
    dat=dat(idx,:);
    
    tData{i}=dat;
    n=length(dat);
    tmx=max(tmx,max(dat(:,3)));
    epMean(i,1)=mean(dat(:,3));
    epMean(i,2)=std(dat(:,3))/sqrt(n);
    epMean(i,3)=n;
end
tk=linspace(0,tmx,20);
d=tk(2)-tk(1);
figure;
for i=1:chamberNum
    subplot(chamberNum,1,i);
    hist(tData{i}(:,3),tk);
    set(gca,'xlim',[-d,tmx+d]);
end
disp('mean/sem/N of episode in each chamber:');
disp(epMean);
%end