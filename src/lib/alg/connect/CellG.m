function CG=CellG(A,ng)                                                    
G=graph(A);                                                                
[bin,binsize] = conncomp(G);                                               
b=find(binsize>=ng);                                                        
[~,m]=size(b);                                                             
CG=cell(m,1);
if m>0
    for i=1:m
        [~,com]=find(bin==b(i));                                           
        CG(i,1)={com};
    end
end
end