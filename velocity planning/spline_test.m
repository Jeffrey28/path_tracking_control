% cubic spline ¿¹Á¦

WP=load('Path_rotated.txt');

length=30;

x_data=zeros(length,1);
for i=1:length
    if i==1
        x_data(i)=sqrt((WP(i+1,1)-WP(i,1))^2+(WP(i+1,2)-WP(i,2))^2);
    else
        x_data(i)=x_data(i-1) + sqrt((WP(i+1,1)-WP(i,1))^2+(WP(i+1,2)-WP(i,2))^2);
    end
end
y_data=WP(1:length,3);

cubic_spline(x_data,y_data,10);