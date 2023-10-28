%remove the small segments 
function state=smallsegRemove(state,npt,opTag)
d=0*state;
d(2:end)=state(2:end)-state(1:end-1);
idx0=find(d~=0);
dd=idx0(2:end)-idx0(1:end-1);
ix=find(dd<=npt);
if ~isempty(ix)
	for k=1:length(ix)
        if state(idx0(ix(k))-1)~=opTag
            state(idx0(ix(k)):(idx0(ix(k)+1)-1))=state(idx0(ix(k))-1);
        end
	end
end