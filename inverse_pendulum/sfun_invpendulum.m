function [sys,x0,str,ts] = sfun_invpendulum(t,x,u,flag)
%%
switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 1
        sys = mdlDerivatives(t,x,u);
    case 3
        sys = x;
    case {2,4,9}
        sys = [];
    otherwise
        error(['Unhandled flag = ', num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes
%%
sizes = simsizes;
sizes.NumContStates  = 4;   %连续状态个数
sizes.NumDiscStates  = 0;   %离散状态个数
sizes.NumOutputs     = 4;   %输出个数
sizes.NumInputs      = 2;   %输入个数
sizes.DirFeedthrough = 0;   %是否直接馈入
sizes.NumSampleTimes = 1;   %采样时间个数，至少一个
sys = simsizes(sizes);     %将size结构传到sys中

x0  = [0.5;0;0;0];                     %初始状态向量，由传入的参数决定，没有为空
str = [];
ts  = [0 0];                  %设置采样时间

function sys = mdlDerivatives(t,x,u)
%%
if t>0&&t<5
    u(1) = 0.2*sin(t);
else
    u(1) = 0;
end

%控制器
% K = [32.9475,1.0964,8.4524,2.5078];
% K = [61.489011865719810,5.259929397723181,13.004204855500932,8.775208623371093];
K = [2.481541933547076e+03,4.619949761250870e+02,6.252244105230664e+02,6.353061287699737e+02];
u(2) = K*x;


%------对象描述------------
% theta = x(1);
% shift = x(2);
% dtheta = x(3);
% dshift = x(4);
% noise = u(1);
% inpsig = u(2);

M=1; %小车质量
m=0.1; %倒立摆质量
L=0.5; %摆的长度
g=9.8 ;%重力加速度
J=m*L^2/3; %转动惯量

k=J*(M+m)+m*M*L^2 ;
k1=(M+m)*m*g*L/k ;
k2=-m^2*g*L^2/k ;
k3=-m*L/k ;
k4=( J+m*L^2 ) /k ; %中间变量

A=[0,0,1,0 ;
    0,0,0,1 ;
    k1,0,0,0;
    k2,0,0,0];
B1=[0;0;0.2;0.2] ;
B2=[0; 0; k3; k4];
dx = A*x+B1*u(1)+B2*u(2);
sys = dx;
