%% Parameters
Rated_Voltage = 24;
Rated_Current=	9;
Stall_Current =	24.5;
No_Load_Current	= 700e-3;
No_load_speed =330 * 2* pi /60;
Rated_speed	= 250 * 2 * pi / 60;
Rated_torque =	2.94;
%% Assumptions
Inductance = 0;
Inertia = 0.02;
%% Calculations
Armature_Resistance = Rated_Voltage/ Stall_Current;
Electric_Coefficient = (Rated_Voltage-No_Load_Current*Armature_Resistance)/No_load_speed;
Torque_Coefficient = Electric_Coefficient;
Friction = Rated_torque/Rated_speed;
%% modelling
system = tf(Torque_Coefficient,[(Inertia*Armature_Resistance) (Friction*Armature_Resistance+Electric_Coefficient*Torque_Coefficient)])