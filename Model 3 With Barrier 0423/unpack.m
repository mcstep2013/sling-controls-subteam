function varargout=unpack(arrIn)
if nargout~=length(arrIn)
   error('Youre using this wrong.') 
end
for k = 1:length(arrIn)
    varargout{k} = arrIn(k);
end
end