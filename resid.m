%calculate the residual times in each chamber
function resid(folder,tag)
folder=AddEnd2Folder(folder);
extName='*.mat';
%get the file list
duDat1=[];
duDat2=[];
flist=dir(strcat(folder,extName));
fnum=length(flist);
if fnum>0
    tnum=0;
    for i=1:fnum
        fname=strcat(folder,flist(i).name);
        tDat=getSingle(fname);
        if tag(i)==1
            duDat1=[duDat1;tDat{1}];
            duDat2=[duDat2;tDat{2}];
        elseif tag(i)==2
            duDat1=[duDat1;tDat{2}];
            duDat2=[duDat2;tDat{1}];
        end
    end
else
    disp('No file found!');
end
disp(duDat1(:,3));
disp('--------------');
disp(duDat2(:,3));
end

function tData=getSingle(filename)
load(filename);
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
    idx=find(dat(2:end-1,3)<1);
    if ~isempty(idx)
        idx=idx+1;
        %previous
        idx2a=find(dat(idx,1)-dat(idx-1,2)<1);
        if ~isempty(idx2a)
            dat(idx(idx2a)-1,2)=dat(idx(idx2a),2);
            dat(idx(idx2a)-1,3)=dat(idx(idx2a)-1,2)-dat(idx(idx2a)-1,1);
        end
        %next
        idx2b=find(dat(idx+1,1)-dat(idx,2)<1);
        if ~isempty(idx2b)
            dat(idx(idx2b)+1,1)=dat(idx(idx2b),1);
            dat(idx(idx2b)+1,3)=dat(idx(idx2b)+1,2)-dat(idx(idx2b)+1,1);
        end 
        idx2=[idx2a;idx2b];
        idx3=setdiff(1:n,idx(idx2));
        dat=dat(idx3,:);
    end
    
    n=length(dat);
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
%tk=linspace(0,tmx,20);
%d=tk(2)-tk(1);
% figure;
% for i=1:chamberNum
%     subplot(chamberNum,1,i);
%     hist(tData{i}(:,3),tk);
%     set(gca,'xlim',[-d,tmx+d]);
% end
%disp('mean/sem/N of episode in each chamber:');
%disp(epMean);
end

function fdname=AddEnd2Folder(folder)
%add the terminal if necessary
if ispc
    str0='\';
else
    str0='/';
end
if folder(end)~=str0
    fdname=strcat(folder,str0);
else
    fdname=folder;
end
end