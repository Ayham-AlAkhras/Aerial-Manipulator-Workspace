clc
clear
close all

maxangle=pi;
%Limits, from physical design
th1_1=-120*pi/180;
th1_2=-10*pi/180;
th2_1=0;
th2_2=140*pi/180;
th2a_1 = -90*pi/180;
th2a_2 = pi*6/7+th2a_1;

res=1*pi/180;

%Masses
m_d=1;              %Drone
m_b=0.545;          %Battery
m_l1=0.053;         %Link 1
m_l2=0.035;         %Link 2
m_p=0.2;            %Payload

%Link lengths
l1=0.15;
l2=0.14;

%COG locations
cog_l1=0.08;
cog_l2=0.082;

%Brute force end effector location computation
n=1;
nmax=round((th1_2-th1_1)/res+1);
x_p(nmax)=0;
y_p(nmax)=0;
x_b(nmax)=0;
for th1=th1_1:res:th1_2
    for th2=th2_1:res:th2_2
        
          th2a=th1+th2;
        if th2a>=th2a_1 && th2a<=th2a_2
        %%Forward kinematics 
        x_l1=l1*cos(th1);                   %%Link length
        x_l2=l1*cos(th1)+l2*cos(th2a);
        y_l1=l1*sin(th1);
        y_l2=l1*sin(th1)+l2*sin(th2a);
        x_p(n)=l1*cos(th1)+l2*cos(th2a);        %%Payload location
        y_p(n)=l1*sin(th1)+l2*sin(th2a);
        xm_l1=cog_l1*cos(th1);                           %%Link COM
        xm_l2=l1*cos(th1)+cog_l2*cos(th2a);
        ym_l1=cog_l1*sin(th1);
        ym_l2=l1*sin(th1)+cog_l2*sin(th2a);
        x_b(n)=-(m_l1*xm_l1+m_l2*xm_l2+m_p*x_p(n))/m_b;
        n=n+1;
        end
    end
end
c=y_p<=0;
x_p=x_p(c);
y_p=y_p(c);
d=x_p>=0;
x_p=x_p(d);
y_p=y_p(d);

b=boundary(x_p',y_p',1);
scatter(x_p,y_p,'.');
hold on

scatter (x_b,zeros(size(x_b)),'g.');
plot (0,0,'ro');
x_p_b=x_p(b);
y_p_b=y_p(b);
axis equal
xlabel('X(m)')
ylabel('Y(m)')
legend('Workspace','Battery Slider Location','Origin')
figure
plot(x_p_b,y_p_b);

nmax=round((maxangle-0)/res+1);
sb=size(b);
x(nmax,sb(1))=0;
y(nmax,sb(1))=0;
z(nmax,sb(1))=0;
i=1;
    for theta=0:res:maxangle
        r=sqrt(x_p_b.^2+y_p_b.^2);
        phi=acos(x_p_b./r);
        [x(i,:),y(i,:),z(i,:)]=sph2cart(theta,-phi,r);
        i=i+1;
    end
% xv=x(:);
% yv=y(:);
% zv=z(:);
% b2=boundary(xv,yv,zv,1); % This part takes a while!
% xb2=b2(:,1);
% yb2=b2(:,2);
% zb2=b2(:,3);
figure
view (3)
hold on
grid on
axis equal
mesh(x,y,z);
% plot3(0,0,0,'ro')
% scatter3(zeros(size(x_b)),x_b,zeros(size(x_b)),'g.');
% data=[xb2,yb2,zb2];
xlabel('X(m)')
ylabel('Z(m)')
zlabel('Y(m)')
% smooth3(data')