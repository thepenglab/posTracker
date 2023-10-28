%load the data from .mat and showTrack
function mat2Track(filename)
load(filename);
dur=50*60;
[vHei,vWid,chs]=size(backgroundImg);
handle=figure('position',[100,100,vWid,vHei]);  
set(gca,'position',[0,0,1,1]);
axis image off;
whitebk=backgroundImg*0+255;
imagesc(whitebk);
if ~isempty(ROI.x)
    n=length(ROI.x);
    w0=min(ROI.width);
    c1=[0.5,0,0];
    c2=[0.75,0.75,0.75];
    cls=get(gca,'colororder');
    for i=1:n
        rectangle('Position',[ROI.x(i),ROI.y(i),ROI.width(i),ROI.height(i)],...
                'LineStyle','--','edgecolor',cls(i+1,:));
            
        %horizontal
        rectangle('position',[ROI.x(i),ROI.y(i)-12,w0*scoreData(i)+0.01,10],...
            'EdgeColor',c1,'FaceColor', c1);
        text(ROI.x(i),ROI.y(i)-6,num2str(round(scoreData(i)*100)/100),'color',c2);        
    end
end
hold all;
idx=find(trackData(:,1)<dur);
plot(trackData(idx,2),trackData(idx,3),'-k','LineWidth',1);
axis off;
end