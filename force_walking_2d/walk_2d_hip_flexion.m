% WALKING 2D PREDICTION: MINIMISE HIP FLEXION
%
% @author: Prasanna Sritharan


import org.opensim.modeling.*;


%% DEFINE OPTIMAL CONTROL PROBLEM


% Create MocoStudy
study = MocoStudy();
study.setName('walk_2d_hip_flexion');


% Get the model
model_processor = ModelProcessor('2D_gait.osim');
model_processor.append(ModOpTendonComplianceDynamicsModeDGF('implicit'));
%model_processor.append(ModOpAddReserves())

% Get the MocoProblem and set the model on it
problem = study.updProblem();
problem.setModelProcessor(model_processor);



%% SET GOALS


% Unilateral periodicity:
% All states are periodic except pevlix_tx value, lumbar control is periodic
periodicity_goal = MocoPeriodicityGoal('periodicity');
model = model_processor.process();
model.initSystem();
for sn=0:(model.getNumStateVariables()-1)
    state_name = string(model.getStateVariableNames().getitem(sn));
    if ~contains(state_name, 'pelvis_tx/value')
        periodicity_goal.addStatePair(MocoPeriodicityGoalPair(state_name));
    end
end
periodicity_goal.addControlPair(MocoPeriodicityGoalPair('/lumbarAct'));
problem.addGoal(periodicity_goal);

% Average gait speed:
speed_goal = MocoAverageSpeedGoal('average_speed');
speed_goal.set_desired_average_speed(1.0);
problem.addGoal(speed_goal);

% Hip joint angles:
hip_goal = MocoSumSquaredStateGoal();
hip_goal.setPattern("/jointset/hip_r/hip_flexion_r/value");
problem.addGoal(hip_goal);

% Control effort:
% Minimise overall control effort
effort_goal = MocoControlGoal('effort', 10);
effort_goal.setExponent(3);
effort_goal.setDivideByDisplacement(true);
problem.addGoal(effort_goal);



%% BOUNDS


% Set time bounds
problem.setTimeBounds(0.0, 0.94);

% Coordinate bounds
coord_bounds = {{'/jointset/groundPelvis/pelvis_tilt/value', [-20*pi/180, 20*pi/180]}, ...
                {'/jointset/groundPelvis/pelvis_tx/value', [0, 2]}, ...
                {'/jointset/groundPelvis/pelvis_ty/value', [0.75, 1.25]}, ...
                {'/jointset/hip_r/hip_flexion_r/value', [-10*pi/180, 60*pi/180]}, ...
                {'/jointset/hip_l/hip_flexion_l/value', [-10*pi/180, 60*pi/180]}, ...
                {'/jointset/knee_r/knee_angle_r/value', [-50*pi/180, 0]}, ...
                {'/jointset/knee_l/knee_angle_l/value', [-50*pi/180, 0]}, ...
                {'/jointset/ankle_r/ankle_angle_r/value', [-15*pi/180, 25*pi/180]}, ...
                {'/jointset/ankle_l/ankle_angle_l/value', [-15*pi/180, 25*pi/180]}, ...
                {'/jointset/lumbar/lumbar/value', [0, 20*pi/180]}};

% Set coordinate bounds
for bnd=coord_bounds
    problem.setStateInfo(bnd{1}{1}, bnd{1}{2});
end



%% SOLVE


% Configure the solver
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(100);
solver.set_verbosity(2);
solver.set_optim_convergence_tolerance(1e-4);
solver.set_optim_constraint_tolerance(1e-4);
solver.set_optim_max_iterations(5000);

% Set the initial guess to be the symmetric two-steps tracking solution
% from example2DWalking.py. Run this first, or proceed without a guess.
guess_file = 'walk_2D_two_steps_tracking_solution.sto';
if isfile(guess_file)
    solver.setGuessFile(guess_file);
end
        
% Print the Study to file
study.print('walking_2d_hip_flexion.omoco');

% Solve
solution = study.solve();
solution.write('walking_2d_hip_flexion_solution.sto');

% Write predicted GRF to file
% contact_r = ['contactHeel_r', 'contactFront_r'];
% contact_l = ['contactHeel_l', 'contactFront_l'];
%contact_forces_table = createExternalLoadsTableForGait(model, solution, contact_r, contact_l);
%osim.STOFileAdapter().write(contact_forces_table, 'walking_2d_hip_force_ground_forces.sto');

% Calculate step time asymmetry
%step_time_asym, step_length_asym = stas.computeStepAsymmetry(model, 25, 'walking_2d_hip_force_solution.sto', 'walking_2d_hip_force_ground_forces.sto')
%print('\nStep time asymmetry: \t%f\n' % step_time_asym)
%print('\nStep length asymmetry:\t%f\n' % step_length_asym)

% Visualise
study.visualize(solution);
