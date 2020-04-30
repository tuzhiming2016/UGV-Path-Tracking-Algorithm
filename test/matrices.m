clear;clc;
a=2;b=3;c=4;N=5;A=zeros(N,1);K=zeros(N,N);C=zeros(N,1);
for i=1:N
    A(i,1)=a^i;
    C(i,1)=0;
    for j=1:N
        if i>=j
            K(i,j)=a^(i-j)*b;
        else
            K(i,j)=0;
        end
    end
    for j=1:i
        C(i,1)=C(i,1)+a^(j-1)*c;
    end
end