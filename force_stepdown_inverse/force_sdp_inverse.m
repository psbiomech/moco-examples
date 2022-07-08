%% FORCE STEP DOWN PIVOT - MINIMISE JOINT FORCE - MOCOINVERSE
% Prasanna Sritharan, July 2022
%
%
% MocoInverse analysis: goal minimise joint forces while undertaking
% prescribed step-down and pivot motion.


import org.opensim.modeling.*;


% Create Inverse analysis
invers = MocoInverse();

% Add ModelProcessor with external loads
modproc = ModelProcessor('FAILT01.osim');
modproc.append(ModOpAddExternalLoads('FAILT01_SDP01_ExternalLoads.xml'));
modproc.append(ModOpIgnoreTendonCompliance());
modproc.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
modproc.append(ModOpIgnorePassiveFiberForcesDGF());
modproc.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
modproc.append(ModOpAddReserves(1.0));
invers.setModel(modproc);

% Add TableProcessor with coordinate data
invers.setKinematics(TableProcessor('FAILT01_SDP01_ik.mot'));
invers.set_kinematics_allow_extra_columns(true);

% Time and mesh interval
invers.set_initial_time(0.94);
invers.set_final_time(2.64);
invers.set_mesh_interval(0.04)

% Get the MocoProblem
study = invers.initialize();
problem = study.updProblem();

% Set the reserve actuator weights in the effort goal
effort = MocoControlGoal.safeDownCast(problem.updGoal('excitation_effort'));
effort.setWeight(0.01);
effort.setWeightForControlPattern('^/forceset/reserve_\S+', 10);
effort.setWeightForControlPattern('^/forceset/reserve_\S+', 10);
effort.setWeightForControl('/forceset/reserve_jointset_ground_pelvis_pelvis_ty', 100);


% Solve the MocoStudy
solution = study.solve();







