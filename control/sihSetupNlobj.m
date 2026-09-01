function sihSetupNlobj()
%SIHSETUPNLOBJ  Build and validate the nlobj MPC controller object needed
%by M4_Control's Nonlinear MPC Controller block. Extracted from
%control/setup_m4_day1.m's controller-build section so it can be called
%from sih_top_model's InitFcn without running that script's full
%standalone demo/plot/clear-workspace logic.

vp = evalin('base', 'vehicleParams();');

nx = 4; nu = 2; ny = 4;
Ts = 0.1;
p  = 20;
c  = 3;

nlobj = nlmpc(nx, ny, nu);
nlobj.Ts                     = Ts;
nlobj.PredictionHorizon      = p;
nlobj.ControlHorizon         = c;
nlobj.Model.StateFcn         = "bicycleStateFcn";
nlobj.Model.IsContinuousTime = true;
nlobj.Jacobian.StateFcn      = "bicycleStateJacobianFcn";

nlobj.MV(1).Min     = -vp.deltaMax;
nlobj.MV(1).Max     =  vp.deltaMax;
nlobj.MV(1).RateMin = -0.10;
nlobj.MV(1).RateMax =  0.10;

nlobj.MV(2).Min     = -5.0;
nlobj.MV(2).Max     =  2.0;
nlobj.MV(2).RateMin = -2.0;
nlobj.MV(2).RateMax =  2.0;

nlobj.OV(4).Min = -3.0;
nlobj.OV(4).Max = 10.0;

nlobj.Weights.OutputVariables          = [10  10  3  1];
nlobj.Weights.ManipulatedVariables     = [0.1 0.1];
nlobj.Weights.ManipulatedVariablesRate = [1.0 0.5];

nlobj.Optimization.UseSuboptimalSolution       = true;
nlobj.Optimization.SolverOptions.MaxIterations = 25;

validateFcns(nlobj, [0;0;0;2], [0;0]);

assignin('base', 'nlobj', nlobj);
fprintf('[M6] nlobj created and validated (via sihSetupNlobj.m).\n');
end
