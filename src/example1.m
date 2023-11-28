clear all
close all

set(0, 'defaultlinelinewidth', 3);
set(0, 'DefaultAxesFontSize', 16);
set(0, 'defaulttextfontsize', 18);


%% Composite Silicon Graphite electrode

%% Import the required modules from MRST
% load MRST modules
mrstModule add ad-core matlab_bgl

%% Shortcuts
% We define shorcuts for the sub-models.

ne  = 'NegativeElectrode';
pe  = 'PositiveElectrode';
co  = 'Coating';
gr = 'ActiveMaterial1'; % Graphite
si = 'ActiveMaterial2'; % Silicon
bd  = 'Binder';
ad  = 'ConductingAdditive';
sd  = 'SolidDiffusion';
itf = 'Interface';

%% Setup the properties of the battery
%
% We load the property of a composite silicon graphite electrode, see :ref:`compositeElectrode`
%

jsonstruct_composite_material = parseBattmoJson('ParameterData/BatteryCellParameters/LithiumIonBatteryCell/lithium_ion_battery_nmc_silicon_graphite.json');

%%
% For the remaining properties, we consider a standard data set
jsonstruct_cell = parseBattmoJson('ParameterData/BatteryCellParameters/LithiumIonBatteryCell/lithium_ion_battery_nmc_graphite.json');

%%
% We remove form the standard data set the :code:`ActiveMaterial` field. This step is not necessary but is cleaner and
% we avoid a warning.
jsonstruct_cell.(ne).(co) = rmfield(jsonstruct_cell.(ne).(co), 'ActiveMaterial');

%%
% We merge the two json structures
jsonstruct = mergeJsonStructs({jsonstruct_composite_material, ...
                               jsonstruct_cell});

%%
% We do not consider the thermal model and remove the current collector
jsonstruct.use_thermal = false;
jsonstruct.include_current_collectors = false;

%%
% Now, we update the paramobj with the properties of the mesh.
gen = BatteryGeneratorP2D();

fig1 = 1;
fig2 = 99;


%%
% We set the mass fractions of the different material in the coating of the negative electrode. This information could
% have been passed in the json file earlier (:ref:`compositeElectrode`)


% paramobj.(ne).(co).(am1).massFraction = 0.9;
% paramobj.(ne).(co).(am2).massFraction = 0.08;
% paramobj.(ne).(co).(bd).massFraction  = 0.01;
% paramobj.(ne).(co).(ad).massFraction  = 0.01;

Gr = [0.8, 0.9];
Si = [0.05, 0.08];
[tmpa, tmpb] = meshgrid(Gr, Si);
mfCases = [tmpa(:), tmpb(:)];

mfCases = [0.9 0.08;
           0.8 0.18;
           0.7 0.28];

for k = 1:size(mfCases, 1)

    paramobj = BatteryInputParams(jsonstruct);
    paramobj.Control.CRate = 0.5;

    grmf = mfCases(k, 1);
    simf = mfCases(k, 2);
    bdmf = (1 - simf - grmf)/2;
    admf = 1 - simf - grmf - bdmf;
    assert(abs(grmf + simf + bdmf + admf - 1) < 1e-15);

    simtag = sprintf('%g %g', simf, grmf);

    paramobj.(ne).(co).(si).massFraction = simf;
    paramobj.(ne).(co).(gr).massFraction = grmf;
    paramobj.(ne).(co).(bd).massFraction = bdmf;
    paramobj.(ne).(co).(ad).massFraction = admf;

    % paramobj.(ne).(co).(gr).(itf).saturationConcentration =  28700;
    % paramobj.(ne).(co).(si).(itf).saturationConcentration = 278000;

    paramobj = gen.updateBatteryInputParams(paramobj);

    model = Battery(paramobj);

    % Schedule
    CRate = model.Control.CRate;

    total = 1.1*hour/CRate;

    n  = 100;
    dt = total/n;
    step = struct('val', dt*ones(n, 1), 'control', ones(n, 1));

    tup = 0.1; % rampup value for the current function, see rampupSwitchControl
    srcfunc = @(time, I, E) rampupSwitchControl(time, tup, I, E, ...
                                                model.Control.Imax, ...
                                                model.Control.lowerCutoffVoltage);
    control = struct('src', srcfunc, 'IEswitch', true);

    schedule = struct('control', control, 'step', step);

    % Initial state
    initstate = model.setupInitialState();

    % initstate.(ne).(co).(si).(sd).c = 277000*ones(size(initstate.(ne).(co).(si).(sd).c));

    % Nonlinear solver
    nls = NonLinearSolver();
    nls.maxIterations = 10;
    nls.errorOnFailure = false;
    nls.timeStepSelector = StateChangeTimeStepSelector('TargetProps', {{'Control','E'}}, 'targetChangeAbs', 0.03);
    model.nonlinearTolerance = 1e-3*model.Control.Imax;
    model.verbose = false;

    % Run
    [~, states, ~] = simulateScheduleAD(initstate, model, schedule, 'OutputMinisteps', true, 'NonLinearSolver', nls);

    %% Plot
    for istate = 1 : numel(states)
        states{istate} = model.evalVarName(states{istate}, {ne, co, 'SOC'});
    end
    E    = cellfun(@(x) x.Control.E, states);
    I    = cellfun(@(x) x.Control.I, states);
    time = cellfun(@(x) x.time, states);
    SOC  = cellfun(@(x) x.(ne).(co).SOC, states);
    SOC1 = cellfun(@(x) x.(ne).(co).(gr).SOC, states);
    SOC2 = cellfun(@(x) x.(ne).(co).(si).SOC, states);

    %figure(fig1); hold on
    figure; hold on

    plot(time/hour, SOC, 'displayname', 'Accumulated');
    grid on
    plot(time/hour, SOC1, 'displayname', 'Gr');
    grid on
    plot(time/hour, SOC2, 'displayname', 'Si');
    grid on

    xlabel('Time / h');
    ylabel('SOC / -');
    title(sprintf('SOCs %s', simtag))
    legend show

    figure(fig2); hold on
    plot(time/hour, E, 'displayname', simtag)
    xlabel('Time / h');
    ylabel('Potential / V');
    grid on
    leg2 = legend('show');

end

title(leg2, 'Si Gr');

return

dischargeStates = states;

%% Setup charge schedule

%%
% We use the last computed state of the discharge as the initial state for the charge period.
initstate = states{end};

%%
% We use a new control. Note the minus sign in front of :code:`model.Control.Imax`
srcfunc = @(time, I, E) rampupSwitchControl(time, tup, I, E, ...
                                            -model.Control.Imax, ...
                                            model.Control.upperCutoffVoltage);
control = struct('src', srcfunc, 'IEswitch', true);
schedule = struct('control', control, 'step', step);

%% Run the simulation for the charge perios
[wellSols, states, report] = simulateScheduleAD(initstate, model, schedule, 'OutputMinisteps', true, 'NonLinearSolver', nls);

chargeStates = states;

%% Visualisation

%%
% We concatenate the states we have computed
allStates = vertcat(dischargeStates, chargeStates);

%%
% Some ploting setup

%%
% We extract the voltage, current and time from the simulation output
E    = cellfun(@(x) x.Control.E, allStates);
I    = cellfun(@(x) x.Control.I, allStates);
time = cellfun(@(x) x.time, allStates);

%%
%  We plot the voltage and current
figure
subplot(2, 1, 1);
plot(time/hour, E);
xlabel('Time / h');
ylabel('Voltage / V');
title('Voltage')
grid on
subplot(2, 1, 2);
plot(time/hour, I);
xlabel('Time / h');
ylabel('Current / I');
title('Current')
grid on

%%
% We compute and plot the state of charges in the different material

figure
hold on

for istate = 1 : numel(allStates)
    allStates{istate} = model.evalVarName(allStates{istate}, {ne, co, 'SOC'});
end

SOC  = cellfun(@(x) x.(ne).(co).SOC, allStates);
SOC1 = cellfun(@(x) x.(ne).(co).(si).SOC, allStates);
SOC2 = cellfun(@(x) x.(ne).(co).(am2).SOC, allStates);

plot(time/hour, SOC, 'displayname', 'SOC - cumulated');
grid on
plot(time/hour, SOC1, 'displayname', 'SOC - Graphite');
grid on
plot(time/hour, SOC2, 'displayname', 'SOC - Silicon');
grid on

xlabel('Time / h');
ylabel('SOC / -');
title('SOCs')

legend show
