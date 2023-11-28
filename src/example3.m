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
gr  = 'ActiveMaterial1'; % Graphite
si  = 'ActiveMaterial2'; % Silicon
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

jsonstruct.Control.rampupTime = 1;
jsonstruct.Control.lowerCutoffVoltage = 3.5;

%%
% Now, we update the paramobj with the properties of the mesh.
gen = BatteryGeneratorP2D();
gen.xlength(4) = 1e-2;

gen.fac = 10;
gen = gen.applyResolutionFactors();

fig1 = 1;
fig2 = 99;

% silicon volume fractions
vfCases = [0.001; 0.04; 0.1];

bdvf = 0.01;
advf = 0.01;

%% Setup current reference
%
% We setup a reference model and recover the current model.Control.Imax
%

CRate = 0.5;

paramobj = BatteryInputParams(jsonstruct);
paramobj.Control.CRate = 0.5;

sivf = vfCases(3);
grvf = 1 - sivf - bdvf - advf;
assert(abs(sivf + grvf + bdvf + advf - 1) < 1e-15);

compnames = {gr, si, bd, ad};
for icomp = 1 : numel(compnames)
    compname = compnames{icomp};
    compInds.(compname) = icomp;
end

volumefractions = nan(4, 1);
volumefractions(compInds.(si)) = sivf;
volumefractions(compInds.(gr)) = grvf;
volumefractions(compInds.(bd)) = bdvf;
volumefractions(compInds.(ad)) = advf;

paramobj.(ne).(co).volumeFractions = volumefractions;
paramobj.(ne).(co).volumeFraction = 0.75;

% paramobj.(ne).(co).(gr).(itf).saturationConcentration =  28700;
% paramobj.(ne).(co).(si).(itf).saturationConcentration = 278000;

% From Chen and/or https://www.sciencedirect.com/science/article/pii/S0378775322001604, doi:https://doi.org/10.1016/j.jpowsour.2022.231142
paramobj.(ne).(co).(gr).(sd).referenceDiffusionCoefficient = 5.5e-14;
paramobj.(ne).(co).(si).(sd).referenceDiffusionCoefficient = 1.67e-14;

paramobj.(ne).(co).(gr).(sd).particleRadius = 5.86*micro*meter;
paramobj.(ne).(co).(si).(sd).particleRadius = 1.52*micro*meter;

paramobj = gen.updateBatteryInputParams(paramobj);

model = Battery(paramobj);

Iref = model.Control.Imax;

[a, v] = computeCellCapacity(model);

for k = 1 : size(vfCases, 1)

    paramobj = BatteryInputParams(jsonstruct);

    sivf = vfCases(k, 1);
    %grvf = vfCases(k, 2);
    grvf = 1 - sivf - bdvf - advf;
    assert(abs(sivf + grvf + bdvf + advf - 1) < 1e-15);

    simtag = sprintf('Silicon: %g, Graphite: %g', sivf, grvf);
    fprintf('\n %s \n\n', simtag);

    volumefractions = nan(4, 1);
    volumefractions(compInds.(si)) = sivf;
    volumefractions(compInds.(gr)) = grvf;
    volumefractions(compInds.(bd)) = bdvf;
    volumefractions(compInds.(ad)) = advf;

    paramobj.(ne).(co).volumeFractions = volumefractions;
    paramobj.(ne).(co).volumeFraction = 0.75;
    
    % paramobj.(ne).(co).(gr).(itf).saturationConcentration =  28700;
    % paramobj.(ne).(co).(si).(itf).saturationConcentration = 278000;

    % From Chen and/or https://www.sciencedirect.com/science/article/pii/S0378775322001604, doi:https://doi.org/10.1016/j.jpowsour.2022.231142
    paramobj.(ne).(co).(gr).(sd).referenceDiffusionCoefficient = 5.5e-14;
    paramobj.(ne).(co).(si).(sd).referenceDiffusionCoefficient = 1.67e-14;

    paramobj.(ne).(co).(gr).(sd).particleRadius = 5.86*micro*meter;
    paramobj.(ne).(co).(si).(sd).particleRadius = 1.52*micro*meter;

    paramobj = gen.updateBatteryInputParams(paramobj);

    paramobj.(ne).(co).(gr).(itf).reactionRateConstant = 6.6898e-08;
    paramobj.(ne).(co).(si).(itf).reactionRateConstant = 6.6898e-08;
    
    model = Battery(paramobj);

    % Schedule
    model.Control.Imax = Iref;

    total = 1.3*hour/CRate;

    n  = 100;
    dt = total/n;
    step = struct('val', dt*ones(n, 1), 'control', ones(n, 1));

    srcfunc = model.Control.setupControlFunction();
    stopfunc = model.Control.setupStopFunction();
    control = struct('CCDischarge', true, ...
                     'src', srcfunc     , ...
                     'stopFunction', stopfunc);

    schedule = struct('control', control, 'step', step);

    % Initial state
    initstate = model.setupInitialState();

    % Nonlinear solver
    nls = NonLinearSolver();
    nls.maxIterations = 10;
    nls.errorOnFailure = false;
    nls.timeStepSelector = StateChangeTimeStepSelector('TargetProps', {{'Control','E'}}, 'targetChangeAbs', 0.03);
    model.nonlinearTolerance = 1e-3*model.Control.Imax;
    model.verbose = false;

    % Run
    [~, states, ~] = simulateScheduleAD(initstate, model, schedule, 'OutputMinisteps', true, 'NonLinearSolver', nls);

    ind = cellfun(@(state) ~isempty(state), states);
    states = states(ind);
    
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

