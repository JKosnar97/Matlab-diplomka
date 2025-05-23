% MATLAB Skript pro simulaci kapacity příletů a odletů v TMA Čáslav
% -----------------------------------------------------------------
% Verze 4.6: S prioritními odletovými sloty (striktní varianta)
% Založeno na S1 (verze 4.5) s přidáním prioritních časů pro odlety
% ROZŠÍŘENO O PRIORITNÍ SLOTY PRO ODLETY

clear;
clc;
close all;

% ----------------- KONFIGURAČNÍ PARAMETRY -------------------
% Základní nastavení simulace
SIM_DURATION_S = 3600;            % Doba simulace v sekundách (1 hodina)
TIME_STEP_S = 1;                  % Časový krok pro dynamickou simulaci v sekundách
RANDOM_SEED = 42;                 % Pro reprodukovatelnost výsledků
DRAW_ANIMATION = true;            % Vykreslení animace pohybu letounů
ANIMATE_SPEED = 500;              % Rychlost animace

% Parametry generování letounů
FIXED_NUM_AIRCRAFT = 40;          % Počet příletů
FIXED_NUM_DEPARTURES = 40;        % Počet odletů (včetně prioritních)
ROUTE_PROBABILITY = [0.5, 0.5];   % Pravděpodobnost výběru trasy [IAF SUKAV, IAF PIMEK]

% Parametry separace
MIN_SEPARATION_NM_AIR = 3;        % Minimální separace ve vzduchu 3 NM

% Nová pravidla pro separaci na dráze (časové hodnoty v sekundách)
RUNWAY_OCCUPANCY_AFTER_LANDING_S = 60; 
ARR_AFTER_ARR_MIN_INTERVAL_S = 2 * 60; 
DEP_AFTER_ARR_MIN_INTERVAL_S = 60;     
ARR_AFTER_DEP_AFTER_ARR_MIN_INTERVAL_S = 3 * 60; 

% NOVÉ: Parametry pro prioritní odletové sloty
ENABLE_PRIORITY_DEPARTURE_SLOTS = true;     % Aktivace prioritních slotů
PRIORITY_DEPARTURE_TIMES_S = [600, 1200, 1800, 2400, 3000]; % Prioritní časy (10., 20., 30., 40., 50. minuta)
PRIORITY_PROTECTION_BEFORE_S = 120;         % Ochrana 2 min před prioritním odletem
PRIORITY_PROTECTION_AFTER_S = 90;           % Ochrana 90s po prioritním odletu

% Pracovní zátěž řídícího - 300bodová škála s 70% prahem přetížení
WORKLOAD_INTERVAL_S = 60;         
THEORETICAL_MAX_SCALE = 300;       % Teoretické maximum škály
OVERLOAD_70_PERCENT = 210;         % 70% z 300 = práh přetížení  
MAX_WORKLOAD_POINTS = 210;         % Plánovací limit = práh přetížení
BASE_WORKLOAD_POINTS = 10;         % Základní zátěž
LOW_WORKLOAD_THRESHOLD = 60;       % Nízká zátěž
MEDIUM_WORKLOAD_THRESHOLD = 90;    % Střední zátěž (90-162)
HIGH_WORKLOAD_THRESHOLD = 162;     % Vysoká zátěž (163-210)  
OVERLOAD_THRESHOLD = 210;          % Přetížení (210+) = 70% z 300
CRITICAL_OVERLOAD_THRESHOLD = 250; % Kritické přetížení
EXTREME_OVERLOAD_THRESHOLD = 300;  % Extrémní přetížení (100% škály)

% ----------------- PARAMETRY PRO STATISTIKY -------------------
THEORETICAL_HOURLY_CAPACITY = 60;  % Teoretická hodinová kapacita

% Nastavení generátoru náhodných čísel
rng(RANDOM_SEED);

% ----------------- DEFINICE KOMUNIKAČNÍ ZÁTĚŽE (10% SNÍŽENÍ) -------------------
comm_load_params = struct();
% Snížení o 10% z původních hodnot, zaokrouhleno směrem dolů
comm_load_params.pre_approach_contact.duration_total_transaction_s = 90;
comm_load_params.pre_approach_contact.controller_workload_points = 31; % 35×0.9=31.5→31
comm_load_params.pre_approach_contact.timing_offset_s = -3 * 60; 

comm_load_params.handoff_to_tower.duration_total_transaction_s = 10;
comm_load_params.handoff_to_tower.controller_workload_points = 13; % 15×0.9=13.5→13
comm_load_params.handoff_to_tower.timing_offset_s = 0; 

comm_load_params.tower_radar_coordination.duration_total_transaction_s = 20;
comm_load_params.tower_radar_coordination.controller_workload_points = 22; % 25×0.9=22.5→22
comm_load_params.tower_radar_coordination.timing_offset_s = -1 * 60; 

comm_load_params.post_takeoff_contact.duration_total_transaction_s = 15;
comm_load_params.post_takeoff_contact.controller_workload_points = 22; % 25×0.9=22.5→22
comm_load_params.post_takeoff_contact.timing_offset_s = 0; 

comm_load_params.final_contact.duration_total_transaction_s = 15;
comm_load_params.final_contact.controller_workload_points = 13; % 15×0.9=13.5→13
comm_load_params.final_contact.timing_offset_s = 0;

% ----------------- GEOGRAFICKÉ DEFINICE A TRATĚ -------------------
disp('--- Definice vstupních parametrů, tratí a rychlostních profilů ---');
knots_to_nm_per_sec = 1/3600;
dms_to_dd = @(dms_str) custom_dms2degrees(dms_str); 

coords = struct();
coords.IAF_SUKAV = [dms_to_dd('494045.43N'), dms_to_dd('0153410.49E')];
coords.IAF_PIMEK = [dms_to_dd('494105.71N'), dms_to_dd('0154452.03E')];
coords.FAF = [dms_to_dd('494939N'), dms_to_dd('0153232E')];
coords.RWY31_THR = [dms_to_dd('495553.90N'), dms_to_dd('0152335.81E')];
coords.RWY_ARR = coords.RWY31_THR;
coords.RWY_DEP = coords.RWY31_THR;
coords.ERUSO = [dms_to_dd('501109.07N'), dms_to_dd('0151615.99E')];

dist_IAF_PIMEK_FAF = haversine_distance(coords.IAF_PIMEK(1), coords.IAF_PIMEK(2), coords.FAF(1), coords.FAF(2));
if_rel_pos_pimek = 6.9 / dist_IAF_PIMEK_FAF; 
coords.IF = [coords.IAF_PIMEK(1) + if_rel_pos_pimek * (coords.FAF(1) - coords.IAF_PIMEK(1)), ...
             coords.IAF_PIMEK(2) + if_rel_pos_pimek * (coords.FAF(2) - coords.IAF_PIMEK(2))];

bearing_cv314_rad = (312) * pi/180; 
dist_nm_cv314 = 4.0; earth_radius_nm = 3440.065;
lat_rwy31_thr_rad = coords.RWY31_THR(1) * pi/180; lon_rwy31_thr_rad = coords.RWY31_THR(2) * pi/180;
cv314_lat_rad = asin(sin(lat_rwy31_thr_rad) * cos(dist_nm_cv314/earth_radius_nm) + cos(lat_rwy31_thr_rad) * sin(dist_nm_cv314/earth_radius_nm) * cos(bearing_cv314_rad));
cv314_lon_rad = lon_rwy31_thr_rad + atan2(sin(bearing_cv314_rad) * sin(dist_nm_cv314/earth_radius_nm) * cos(lat_rwy31_thr_rad), cos(dist_nm_cv314/earth_radius_nm) - sin(lat_rwy31_thr_rad) * sin(cv314_lat_rad));
coords.CF314 = [cv314_lat_rad * 180/pi, cv314_lon_rad * 180/pi];

dist_exact = struct();
dist_exact.IAF_SUKAV_IF = haversine_distance(coords.IAF_SUKAV(1), coords.IAF_SUKAV(2), coords.IF(1), coords.IF(2));
dist_exact.IAF_PIMEK_IF = 6.9; 
dist_exact.IF_FAF = haversine_distance(coords.IF(1), coords.IF(2), coords.FAF(1), coords.FAF(2));
dist_exact.FAF_RWY_ARR = haversine_distance(coords.FAF(1), coords.FAF(2), coords.RWY_ARR(1), coords.RWY_ARR(2));
dist_exact.RWY_DEP_CF314 = 5.296; 
dist_exact.CF314_ERUSO = haversine_distance(coords.CF314(1), coords.CF314(2), coords.ERUSO(1), coords.ERUSO(2));

% RYCHLOSTI PŘÍLETŮ A ODLETŮ
speed_profile_arr = struct(); 
speed_profile_arr.IAF = 250; speed_profile_arr.IF = 250; speed_profile_arr.FAF = 210; speed_profile_arr.RWY_ARR = 140; 
speed_profile_dep = struct(); 
speed_profile_dep.RWY_DEP = 0; speed_profile_dep.CF314 = 250; speed_profile_dep.ERUSO = 250; 

t_arr_IAFSUKAV_IF_s = dist_exact.IAF_SUKAV_IF / (mean([speed_profile_arr.IAF, speed_profile_arr.IF]) * knots_to_nm_per_sec);
t_arr_IAFPIMEK_IF_s = dist_exact.IAF_PIMEK_IF / (mean([speed_profile_arr.IAF, speed_profile_arr.IF]) * knots_to_nm_per_sec);
t_arr_IF_FAF_s = dist_exact.IF_FAF / (mean([speed_profile_arr.IF, speed_profile_arr.FAF]) * knots_to_nm_per_sec);
t_arr_FAF_RWY_s = dist_exact.FAF_RWY_ARR / (mean([speed_profile_arr.FAF, speed_profile_arr.RWY_ARR]) * knots_to_nm_per_sec);
route1_total_s = t_arr_IAFSUKAV_IF_s + t_arr_IF_FAF_s + t_arr_FAF_RWY_s;
route2_total_s = t_arr_IAFPIMEK_IF_s + t_arr_IF_FAF_s + t_arr_FAF_RWY_s;
avg_speed_RWY_DEP_CF314 = (speed_profile_dep.RWY_DEP + speed_profile_dep.CF314) / 2; 
td_RWY_DEP_CF314_s = dist_exact.RWY_DEP_CF314 / (avg_speed_RWY_DEP_CF314 * knots_to_nm_per_sec);
speed_CF314_ERUSO = speed_profile_dep.CF314; 
td_CF314_ERUSO_s = dist_exact.CF314_ERUSO / (speed_CF314_ERUSO * knots_to_nm_per_sec);
td_total_s = td_RWY_DEP_CF314_s + td_CF314_ERUSO_s;

disp(['Nové rychlosti: IAF-IF (', num2str(speed_profile_arr.IAF), '-', num2str(speed_profile_arr.IF), '), IF-FAF (', num2str(speed_profile_arr.IF), '-', num2str(speed_profile_arr.FAF), '), FAF-THR (', num2str(speed_profile_arr.FAF), '-', num2str(speed_profile_arr.RWY_ARR), ')']);

route1 = struct('name', 'IAF SUKAV - IF - FAF - THR RWY31', 'waypoints', {{'IAF SUKAV', 'IF', 'FAF', 'RWY_ARR'}}, ...
                'waypoint_coords', [coords.IAF_SUKAV; coords.IF; coords.FAF; coords.RWY_ARR], ...
                'segment_dist', [dist_exact.IAF_SUKAV_IF, dist_exact.IF_FAF, dist_exact.FAF_RWY_ARR], ...
                'segment_time', [t_arr_IAFSUKAV_IF_s, t_arr_IF_FAF_s, t_arr_FAF_RWY_s], ...
                'total_dist', sum([dist_exact.IAF_SUKAV_IF, dist_exact.IF_FAF, dist_exact.FAF_RWY_ARR]), 'total_time', route1_total_s, ...
                'speed_at_waypoints', [speed_profile_arr.IAF, speed_profile_arr.IF, speed_profile_arr.FAF, speed_profile_arr.RWY_ARR]);
route2 = struct('name', 'IAF PIMEK - IF - FAF - THR RWY31', 'waypoints', {{'IAF PIMEK', 'IF', 'FAF', 'RWY_ARR'}}, ...
                'waypoint_coords', [coords.IAF_PIMEK; coords.IF; coords.FAF; coords.RWY_ARR], ...
                'segment_dist', [dist_exact.IAF_PIMEK_IF, dist_exact.IF_FAF, dist_exact.FAF_RWY_ARR], ...
                'segment_time', [t_arr_IAFPIMEK_IF_s, t_arr_IF_FAF_s, t_arr_FAF_RWY_s], ...
                'total_dist', sum([dist_exact.IAF_PIMEK_IF, dist_exact.IF_FAF, dist_exact.FAF_RWY_ARR]), 'total_time', route2_total_s, ...
                'speed_at_waypoints', [speed_profile_arr.IAF, speed_profile_arr.IF, speed_profile_arr.FAF, speed_profile_arr.RWY_ARR]);
route_dep = struct('name', 'THR RWY31 - CV314 - ERUSO', 'waypoints', {{'RWY_DEP', 'CV314', 'ERUSO'}}, ...
                   'waypoint_coords', [coords.RWY_DEP; coords.CF314; coords.ERUSO], ...
                   'segment_dist', [dist_exact.RWY_DEP_CF314, dist_exact.CF314_ERUSO], ...
                   'segment_time', [td_RWY_DEP_CF314_s, td_CF314_ERUSO_s], ...
                   'total_dist', sum([dist_exact.RWY_DEP_CF314, dist_exact.CF314_ERUSO]), 'total_time', td_total_s, ...
                   'speed_at_waypoints', [speed_profile_dep.RWY_DEP, speed_profile_dep.CF314, speed_profile_dep.ERUSO]);
arrival_routes = {route1, route2};

% ----------------- GENEROVÁNÍ A PLÁNOVÁNÍ LETŮ -------------------
disp(' ');
disp('--- Generování a plánování letů ---');
if ENABLE_PRIORITY_DEPARTURE_SLOTS
    disp(['Lety: ', num2str(FIXED_NUM_AIRCRAFT), ' příletů + ', num2str(FIXED_NUM_DEPARTURES), ' odletů (z toho ', num2str(length(PRIORITY_DEPARTURE_TIMES_S)), ' prioritních)']);
else
    disp(['Lety: ', num2str(FIXED_NUM_AIRCRAFT), ' příletů + ', num2str(FIXED_NUM_DEPARTURES), ' odletů, separace: ', num2str(MIN_SEPARATION_NM_AIR), ' NM']);
end

buffer_time = 60;
orig_arrival_times = generate_uniform_arrivals(FIXED_NUM_AIRCRAFT, SIM_DURATION_S, buffer_time);
orig_departure_times = generate_uniform_departures(FIXED_NUM_DEPARTURES, SIM_DURATION_S, buffer_time);

% NOVÁ FUNKCE: Plánování s prioritními sloty
if ENABLE_PRIORITY_DEPARTURE_SLOTS
    [scheduled_flights, rejected_flights_info, final_workload_per_interval, runway_operations_log] = ...
        plan_flights_with_priority_slots(orig_arrival_times, orig_departure_times, ...
        arrival_routes, route_dep, ROUTE_PROBABILITY, SIM_DURATION_S, WORKLOAD_INTERVAL_S, ...
        MAX_WORKLOAD_POINTS, BASE_WORKLOAD_POINTS, comm_load_params, MIN_SEPARATION_NM_AIR, ...
        RUNWAY_OCCUPANCY_AFTER_LANDING_S, ARR_AFTER_ARR_MIN_INTERVAL_S, DEP_AFTER_ARR_MIN_INTERVAL_S, ...
        td_RWY_DEP_CF314_s, PRIORITY_DEPARTURE_TIMES_S, PRIORITY_PROTECTION_BEFORE_S, PRIORITY_PROTECTION_AFTER_S);
else
    % Původní plánování bez priorit
    [scheduled_flights, rejected_flights_info, final_workload_per_interval, runway_operations_log] = ...
        plan_flights_with_complex_workload_control(orig_arrival_times, orig_departure_times, ...
        arrival_routes, route_dep, ROUTE_PROBABILITY, SIM_DURATION_S, WORKLOAD_INTERVAL_S, ...
        MAX_WORKLOAD_POINTS, BASE_WORKLOAD_POINTS, comm_load_params, MIN_SEPARATION_NM_AIR, ...
        RUNWAY_OCCUPANCY_AFTER_LANDING_S, ARR_AFTER_ARR_MIN_INTERVAL_S, DEP_AFTER_ARR_MIN_INTERVAL_S, td_RWY_DEP_CF314_s);
end

scheduled_arrivals = []; scheduled_departures = [];
if ~isempty(scheduled_flights)
    scheduled_arrivals = scheduled_flights(strcmp({scheduled_flights.type}, 'arrival'));
    scheduled_departures = scheduled_flights(strcmp({scheduled_flights.type}, 'departure'));
end

% Počet prioritních a běžných odletů
priority_departures_count = 0;
regular_departures_count = 0;
if ~isempty(scheduled_departures) && isfield(scheduled_departures, 'is_priority')
    priority_departures_count = sum([scheduled_departures.is_priority]);
    regular_departures_count = length(scheduled_departures) - priority_departures_count;
end

disp(['Naplánováno: ', num2str(length(scheduled_arrivals)), ' příletů a ', ...
      num2str(length(scheduled_departures)), ' odletů']);
if ENABLE_PRIORITY_DEPARTURE_SLOTS && priority_departures_count > 0
    disp(['  - Prioritní odlety: ', num2str(priority_departures_count)]);
    disp(['  - Běžné odlety: ', num2str(regular_departures_count)]);
end

% Seznam komunikačních událostí pro výpočet zátěže v dynamické simulaci
aircraft_comm_events_for_dyn_sim = struct('time', {}, 'points', {}, 'flight_id', {}, 'duration', {});
if ~isempty(scheduled_flights)
    for i = 1:length(scheduled_flights)
        flight = scheduled_flights(i);
        flight_id_for_comm = flight.id; 
        if strcmp(flight.type, 'arrival')
            aircraft_comm_events_for_dyn_sim(end+1) = struct('time', flight.entry_time + comm_load_params.pre_approach_contact.timing_offset_s, 'points', comm_load_params.pre_approach_contact.controller_workload_points, 'flight_id', flight_id_for_comm, 'duration', comm_load_params.pre_approach_contact.duration_total_transaction_s);
            aircraft_comm_events_for_dyn_sim(end+1) = struct('time', flight.faf_time + comm_load_params.handoff_to_tower.timing_offset_s, 'points', comm_load_params.handoff_to_tower.controller_workload_points, 'flight_id', flight_id_for_comm, 'duration', comm_load_params.handoff_to_tower.duration_total_transaction_s);
        else 
            aircraft_comm_events_for_dyn_sim(end+1) = struct('time', flight.takeoff_time + comm_load_params.tower_radar_coordination.timing_offset_s, 'points', comm_load_params.tower_radar_coordination.controller_workload_points, 'flight_id', flight_id_for_comm, 'duration', comm_load_params.tower_radar_coordination.duration_total_transaction_s);
            aircraft_comm_events_for_dyn_sim(end+1) = struct('time', flight.cf314_time + comm_load_params.post_takeoff_contact.timing_offset_s, 'points', comm_load_params.post_takeoff_contact.controller_workload_points, 'flight_id', flight_id_for_comm, 'duration', comm_load_params.post_takeoff_contact.duration_total_transaction_s);
            aircraft_comm_events_for_dyn_sim(end+1) = struct('time', flight.eruso_time + comm_load_params.final_contact.timing_offset_s, 'points', comm_load_params.final_contact.controller_workload_points, 'flight_id', flight_id_for_comm, 'duration', comm_load_params.final_contact.duration_total_transaction_s);
        end
    end
    if ~isempty(aircraft_comm_events_for_dyn_sim)
        [~, sort_idx_comm] = sort([aircraft_comm_events_for_dyn_sim.time]);
        aircraft_comm_events_for_dyn_sim = aircraft_comm_events_for_dyn_sim(sort_idx_comm);
    end
end

% ----------------- INICIALIZACE STATISTICKÝCH PROMĚNNÝCH -------------------
active_aircraft_count_per_interval = zeros(1, ceil(SIM_DURATION_S / WORKLOAD_INTERVAL_S));

% ----------------- DYNAMICKÁ SIMULACE A VIZUALIZACE -------------------
disp(' ');
disp('--- Dynamická simulace ---');
landing_count = 0; takeoff_count = 0; 

if DRAW_ANIMATION
    figure('Name', 'TMA Čáslav - Prioritní odletové sloty', 'Position', [50, 50, 1300, 900]);
    hold on;
    
    % Vykreslení tratí a bodů
    plot(coords.IAF_SUKAV(2), coords.IAF_SUKAV(1), 'bp', 'MarkerSize', 10, 'MarkerFaceColor', 'b'); 
    text(coords.IAF_SUKAV(2)+0.005, coords.IAF_SUKAV(1), 'IAF SUKAV','FontSize',9);
    plot(coords.IAF_PIMEK(2), coords.IAF_PIMEK(1), 'bp', 'MarkerSize', 10, 'MarkerFaceColor', 'b'); 
    text(coords.IAF_PIMEK(2)+0.005, coords.IAF_PIMEK(1), 'IAF PIMEK','FontSize',9);
    plot(coords.IF(2), coords.IF(1), 'b^', 'MarkerSize', 8, 'MarkerFaceColor', 'c');
    text(coords.IF(2)+0.005, coords.IF(1), 'IF','FontSize',9);
    plot(coords.FAF(2), coords.FAF(1), 'rd', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    text(coords.FAF(2)+0.005, coords.FAF(1), 'FAF','FontSize',9);
    plot(coords.RWY_ARR(2), coords.RWY_ARR(1), 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'k'); 
    text(coords.RWY_ARR(2)+0.005, coords.RWY_ARR(1)-0.002, 'THR RWY31','FontSize',9);
    plot(coords.CF314(2), coords.CF314(1), 'm>', 'MarkerSize', 8, 'MarkerFaceColor', 'm');
    text(coords.CF314(2)+0.005, coords.CF314(1), 'CV314','FontSize',9);
    plot(coords.ERUSO(2), coords.ERUSO(1), 'mv', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
    text(coords.ERUSO(2)+0.005, coords.ERUSO(1), 'ERUSO','FontSize',9);
    
    plot([coords.IAF_SUKAV(2), coords.IF(2), coords.FAF(2), coords.RWY_ARR(2)], ... 
         [coords.IAF_SUKAV(1), coords.IF(1), coords.FAF(1), coords.RWY_ARR(1)], '--b', 'LineWidth', 1); 
    plot([coords.IAF_PIMEK(2), coords.IF(2), coords.FAF(2), coords.RWY_ARR(2)], ... 
         [coords.IAF_PIMEK(1), coords.IF(1), coords.FAF(1), coords.RWY_ARR(1)], '--c', 'LineWidth', 1);
    plot([coords.RWY_DEP(2), coords.CF314(2), coords.ERUSO(2)], ...
         [coords.RWY_DEP(1), coords.CF314(1), coords.ERUSO(1)], '--m', 'LineWidth', 1);
    
    xlabel('Zeměpisná délka (°E)'); ylabel('Zeměpisná šířka (°N)');
    title_str = ['TMA Čáslav - Separace ', num2str(MIN_SEPARATION_NM_AIR) ,' NM'];
    if ENABLE_PRIORITY_DEPARTURE_SLOTS
        title_str = [title_str, ' (s prioritními sloty)'];
    end
    title(title_str); 
    grid on; axis equal;
    
    % Inicializace grafických objektů pro letadla
    arrival_markers = gobjects(length(scheduled_arrivals), 1);
    arrival_labels = gobjects(length(scheduled_arrivals), 1);
    if ~isempty(scheduled_arrivals)
        for i = 1:length(scheduled_arrivals)
            marker_color_arr = 'b'; if scheduled_arrivals(i).route_index == 2, marker_color_arr = 'c'; end
            arrival_markers(i) = plot(NaN, NaN, 'o', 'MarkerEdgeColor', marker_color_arr, 'MarkerFaceColor', marker_color_arr, 'MarkerSize', 7);
            arrival_labels(i) = text(NaN, NaN, '', 'FontSize', 7, 'Color', marker_color_arr);
        end
    end
    departure_markers = gobjects(length(scheduled_departures), 1);
    departure_labels = gobjects(length(scheduled_departures), 1);
    if ~isempty(scheduled_departures)
        for i = 1:length(scheduled_departures)
            % Prioritní odlety zvýrazněné
            if isfield(scheduled_departures(i), 'is_priority') && scheduled_departures(i).is_priority
                departure_markers(i) = plot(NaN, NaN, 'rs','MarkerEdgeColor','r', 'MarkerFaceColor', 'r', 'MarkerSize', 9);
                departure_labels(i) = text(NaN, NaN, '', 'FontSize', 8, 'Color', 'r', 'FontWeight', 'bold');
            else
                departure_markers(i) = plot(NaN, NaN, 'o','MarkerEdgeColor','m', 'MarkerFaceColor', 'm', 'MarkerSize', 7);
                departure_labels(i) = text(NaN, NaN, '', 'FontSize', 7, 'Color', 'm');
            end
        end
    end
    
    % Indikátory
    time_indicator = annotation('textbox', [0.75, 0.9, 0.2, 0.05], 'String', 'Čas: 00:00:00', 'FitBoxToText','on', 'EdgeColor', 'none', 'FontSize',10);
    sep_indicator = annotation('textbox', [0.05, 0.9, 0.25, 0.05], 'String', 'Min. separace: OK', 'FitBoxToText','on','EdgeColor', 'none', 'FontSize',10, 'BackgroundColor', [0.7 1 0.7]);
    workload_indicator_dyn = annotation('textbox', [0.35, 0.85, 0.35, 0.1], 'String', sprintf('Zátěž: 0 bodů'), 'FitBoxToText','on','EdgeColor', 'none', 'FontSize',10);

    % Struktury pro ukládání aktuálních pozic v dynamické simulaci
    live_arrival_positions = repmat(struct('active', false, 'lat', NaN, 'lon', NaN, 'speed', NaN, 'id', '', 'landed', false), length(scheduled_arrivals), 1);
    live_departure_positions = repmat(struct('active', false, 'lat', NaN, 'lon', NaN, 'speed', NaN, 'id', '', 'exited', false), length(scheduled_departures), 1);
    
    % Nastavení rozsahu os pro lepší zobrazení
    all_lats = [coords.IAF_SUKAV(1), coords.IAF_PIMEK(1), coords.ERUSO(1), coords.RWY_ARR(1)];
    all_lons = [coords.IAF_SUKAV(2), coords.IAF_PIMEK(2), coords.ERUSO(2), coords.RWY_ARR(2)];
    if ~isempty(all_lats) && ~isempty(all_lons)
        axis_buffer = 0.02;
        xlim([min(all_lons)-axis_buffer, max(all_lons)+axis_buffer]);
        ylim([min(all_lats)-axis_buffer, max(all_lats)+axis_buffer]);
    end
    
    current_dynamic_workload_profile = ones(1, ceil(SIM_DURATION_S / WORKLOAD_INTERVAL_S)) * BASE_WORKLOAD_POINTS;

    disp('Spouštím animaci...');
    for current_time_dyn = 0:TIME_STEP_S:SIM_DURATION_S
        % Aktualizace času
        hours = floor(current_time_dyn / 3600); minutes = floor(mod(current_time_dyn, 3600) / 60); seconds = mod(current_time_dyn, 60);
        time_indicator.String = sprintf('Čas: %02d:%02d:%02d', hours, minutes, seconds);
        
        active_flights_now = struct('id',{},'lat',{},'lon',{}); 
        active_aircraft_positions = []; % Pro komplexní model

        % Aktualizace příletů
        for i = 1:length(scheduled_arrivals)
            arr = scheduled_arrivals(i);
            if current_time_dyn >= arr.entry_time && current_time_dyn <= arr.landing_time
                if ~live_arrival_positions(i).active, live_arrival_positions(i).id = arr.id; end
                live_arrival_positions(i).active = true;
                time_in_route = current_time_dyn - arr.entry_time;
                dist_traveled = calculate_distance_traveled_generic_v3(time_in_route, arrival_routes{arr.route_index});
                [lat, lon, speed] = interpolate_position_generic_v3(arrival_routes{arr.route_index}, dist_traveled);
                live_arrival_positions(i).lat = lat; live_arrival_positions(i).lon = lon; live_arrival_positions(i).speed = speed;
                set(arrival_markers(i), 'XData', lon, 'YData', lat, 'Visible', 'on');
                set(arrival_labels(i), 'Position', [lon+0.002, lat+0.002, 0], 'String', sprintf('%s\n%dkt', arr.id, round(speed)), 'Visible', 'on');
                active_flights_now(end+1) = struct('id',arr.id,'lat',lat,'lon',lon);
                active_aircraft_positions(end+1,:) = [lat, lon]; % Pro komplexní model

                if abs(current_time_dyn - arr.landing_time) < TIME_STEP_S && ~live_arrival_positions(i).landed
                    landing_count = landing_count + 1;
                    live_arrival_positions(i).landed = true;
                end
            elseif current_time_dyn > arr.landing_time && live_arrival_positions(i).active
                live_arrival_positions(i).active = false;
                set(arrival_markers(i), 'Visible', 'off'); set(arrival_labels(i), 'Visible', 'off');
            end
        end
        
        % Aktualizace odletů
        for i = 1:length(scheduled_departures)
            dep = scheduled_departures(i);
            if current_time_dyn >= dep.takeoff_time && current_time_dyn <= dep.exit_time
                 if ~live_departure_positions(i).active, live_departure_positions(i).id = dep.id; end
                live_departure_positions(i).active = true;
                time_since_takeoff = current_time_dyn - dep.takeoff_time;
                dist_traveled = calculate_distance_traveled_departure_v3(time_since_takeoff, route_dep, td_RWY_DEP_CF314_s);
                [lat, lon, speed] = interpolate_position_departure_v3(route_dep, dist_traveled, td_RWY_DEP_CF314_s);
                live_departure_positions(i).lat = lat; live_departure_positions(i).lon = lon; live_departure_positions(i).speed = speed;
                set(departure_markers(i), 'XData', lon, 'YData', lat, 'Visible', 'on');
                
                % Zvýraznění prioritních odletů
                if isfield(dep, 'is_priority') && dep.is_priority
                    set(departure_labels(i), 'Position', [lon+0.002, lat+0.002, 0], 'String', sprintf('%s*\n%dkt', dep.id, round(speed)), 'Visible', 'on');
                else
                    set(departure_labels(i), 'Position', [lon+0.002, lat+0.002, 0], 'String', sprintf('%s\n%dkt', dep.id, round(speed)), 'Visible', 'on');
                end
                
                active_flights_now(end+1) = struct('id',dep.id,'lat',lat,'lon',lon);
                active_aircraft_positions(end+1,:) = [lat, lon]; % Pro komplexní model

                if abs(current_time_dyn - dep.takeoff_time) < TIME_STEP_S && ~dep.taken_off 
                    takeoff_count = takeoff_count + 1;
                    scheduled_departures(i).taken_off = true; 
                end
            elseif current_time_dyn > dep.exit_time && live_departure_positions(i).active
                live_departure_positions(i).active = false;
                set(departure_markers(i), 'Visible', 'off'); set(departure_labels(i), 'Visible', 'off');
            end
        end

        % Kontrola separací v reálném čase
        min_current_sep = Inf; conflict_now = false;
        conflicts_detected = false;
        for f1 = 1:length(active_flights_now)
            for f2 = f1+1:length(active_flights_now)
                dist = haversine_distance(active_flights_now(f1).lat, active_flights_now(f1).lon, active_flights_now(f2).lat, active_flights_now(f2).lon);
                min_current_sep = min(min_current_sep, dist);
                if dist < MIN_SEPARATION_NM_AIR
                    conflict_now = true;
                    conflicts_detected = true;
                end
            end
        end
        if conflict_now
            sep_indicator.String = sprintf('KONFLIKT! Min.sep: %.2f NM', min_current_sep);
            sep_indicator.BackgroundColor = [1 0.7 0.7]; 
        elseif ~isinf(min_current_sep)
            sep_indicator.String = sprintf('Min. separace: %.2f NM', min_current_sep);
            sep_indicator.BackgroundColor = [0.7 1 0.7]; 
        else
            sep_indicator.String = 'Žádné aktivní letouny';
            sep_indicator.BackgroundColor = [0.9 0.9 0.9]; 
        end

        % Dynamický výpočet a zobrazení zátěže pro aktuální interval
        current_interval_idx = floor(current_time_dyn / WORKLOAD_INTERVAL_S) + 1;
        if current_interval_idx > length(current_dynamic_workload_profile)
            current_interval_idx = length(current_dynamic_workload_profile); 
        end
        
        % Ukládání počtu aktivních letounů pro statistiky
        if current_interval_idx <= length(active_aircraft_count_per_interval)
            active_aircraft_count_per_interval(current_interval_idx) = length(active_flights_now);
        end
        
        % Získání aktuálních komunikačních událostí
        current_comm_events = struct('time', {}, 'points', {}, 'flight_id', {}, 'duration', {}); 
        for k_comm = 1:length(aircraft_comm_events_for_dyn_sim)
            if aircraft_comm_events_for_dyn_sim(k_comm).time >= (current_interval_idx-1)*WORKLOAD_INTERVAL_S && ...
               aircraft_comm_events_for_dyn_sim(k_comm).time < current_interval_idx*WORKLOAD_INTERVAL_S
                current_comm_events(end+1) = aircraft_comm_events_for_dyn_sim(k_comm);
            end
        end
        
        % Výpočet komunikačního backlogu (zjednodušený)
        comm_backlog = max(0, length(current_comm_events) - 2);
        
        % KOMPLEXNÍ VÝPOČET ZÁTĚŽE podle původního modelu
        if ~isempty(active_aircraft_positions)
            current_dynamic_workload_profile(current_interval_idx) = ...
                calculate_complete_workload_v4(active_aircraft_positions, current_comm_events, ...
                                              current_interval_idx, conflicts_detected, comm_backlog, ...
                                              BASE_WORKLOAD_POINTS, WORKLOAD_INTERVAL_S);
        else
            current_dynamic_workload_profile(current_interval_idx) = BASE_WORKLOAD_POINTS;
        end
        
        workload_val_disp = current_dynamic_workload_profile(current_interval_idx);
        workload_status_str = 'Nízká'; wl_bg_color = [0.7 1 0.7]; 
        scale_percent = (workload_val_disp / THEORETICAL_MAX_SCALE) * 100;
        if workload_val_disp > OVERLOAD_THRESHOLD
            workload_status_str = 'PŘETÍŽENÍ'; wl_bg_color = [1 0.5 0.3]; 
        elseif workload_val_disp > HIGH_WORKLOAD_THRESHOLD
            workload_status_str = 'Vysoká zátěž'; wl_bg_color = [1 0.8 0.4]; 
        elseif workload_val_disp > MEDIUM_WORKLOAD_THRESHOLD
            workload_status_str = 'Střední zátěž'; wl_bg_color = [0.85 1 0.7]; 
        end
        workload_indicator_dyn.String = sprintf('Zátěž: %.0f/300 (%.1f%%)\n%s', workload_val_disp, scale_percent, workload_status_str);
        workload_indicator_dyn.BackgroundColor = wl_bg_color;

        drawnow;
        pause(TIME_STEP_S / ANIMATE_SPEED);
    end
    disp('Animace dokončena.');
else
    disp('Animace vypnuta.');
    landing_count = length(scheduled_arrivals); 
    takeoff_count = length(scheduled_departures);
    
    % Vypočítat počty aktivních letounů pro statistiku
    active_aircraft_count_per_interval = zeros(1, ceil(SIM_DURATION_S / WORKLOAD_INTERVAL_S));
    
    for interval_idx = 1:length(active_aircraft_count_per_interval)
        interval_start = (interval_idx - 1) * WORKLOAD_INTERVAL_S;
        interval_end = interval_idx * WORKLOAD_INTERVAL_S;
        interval_mid = (interval_start + interval_end) / 2;
        
        count = 0;
        for i = 1:length(scheduled_flights)
            flight = scheduled_flights(i);
            if strcmp(flight.type, 'arrival')
                if interval_mid >= flight.entry_time && interval_mid <= flight.landing_time
                    count = count + 1;
                end
            else % departure
                if interval_mid >= flight.takeoff_time && interval_mid <= flight.exit_time
                    count = count + 1;
                end
            end
        end
        active_aircraft_count_per_interval(interval_idx) = count;
    end
end

% ----------------- STATISTICKÉ VYHODNOCENÍ -------------------
disp(' ');
disp('--- Statistické vyhodnocení ---');
disp(['Celkem dokončeno: ', num2str(landing_count), ' přistání a ', num2str(takeoff_count), ' vzletů']);

% NOVÉ - Detailní statistiky pohybů
total_movements = landing_count + takeoff_count;
arrivals_per_hour = landing_count / (SIM_DURATION_S / 3600);
departures_per_hour = takeoff_count / (SIM_DURATION_S / 3600);

disp(['Celkem pohybů: ', num2str(total_movements)]);
disp(['Příletů za hodinu: ', num2str(arrivals_per_hour, '%.2f')]);
disp(['Odletů za hodinu: ', num2str(departures_per_hour, '%.2f')]);

effective_hourly_capacity = (landing_count + takeoff_count) / (SIM_DURATION_S / 3600);
disp(['Efektivní hodinová kapacita: ', num2str(effective_hourly_capacity, '%.2f'), ' pohybů/hod']);
disp(' ');
disp('--- Statistické vyhodnocení ---');
disp(['Celkem dokončeno: ', num2str(landing_count), ' přistání a ', num2str(takeoff_count), ' vzletů']);

effective_hourly_capacity = (landing_count + takeoff_count) / (SIM_DURATION_S / 3600);
disp(['Efektivní hodinová kapacita: ', num2str(effective_hourly_capacity, '%.2f'), ' pohybů/hod']);

% Statistiky prioritních slotů
if ENABLE_PRIORITY_DEPARTURE_SLOTS && priority_departures_count > 0
    fprintf('\n--- STATISTIKY PRIORITNÍCH SLOTŮ ---\n');
    fprintf('Prioritní odletové časy: ');
    for pt = PRIORITY_DEPARTURE_TIMES_S
        fprintf('%d min ', pt/60);
    end
    fprintf('\n');
    fprintf('Naplánováno prioritních odletů: %d z %d\n', priority_departures_count, length(PRIORITY_DEPARTURE_TIMES_S));
    fprintf('Naplánováno běžných odletů: %d\n', regular_departures_count);
    
    % Analýza odmítnutých letů kvůli prioritním slotům
    rejected_due_to_priority = 0;
    if ~isempty(rejected_flights_info)
        for r = 1:length(rejected_flights_info)
            if contains(rejected_flights_info(r).reason, 'priorit', 'IgnoreCase', true)
                rejected_due_to_priority = rejected_due_to_priority + 1;
            end
        end
    end
    fprintf('Odmítnuto kvůli prioritním slotům: %d letů\n', rejected_due_to_priority);
end

% Použijeme `final_workload_per_interval` z plánovací fáze pro finální graf,
% nebo `current_dynamic_workload_profile` pokud byla animace aktivní
workload_data_for_graph = final_workload_per_interval; 
if DRAW_ANIMATION && exist('current_dynamic_workload_profile', 'var')
    workload_data_for_graph = current_dynamic_workload_profile;
end

% ----------------- ROZŠÍŘENÉ STATISTICKÉ ANALÝZY -------------------
disp(' ');
disp('--- Výpočet rozšířených statistik ---');

% KOMUNIKAČNÍ STATISTIKY
total_comm_time_s = 0;
unique_comm_periods = [];

if ~isempty(aircraft_comm_events_for_dyn_sim)
    for i = 1:length(aircraft_comm_events_for_dyn_sim)
        comm_event = aircraft_comm_events_for_dyn_sim(i);
        total_comm_time_s = total_comm_time_s + comm_event.duration;
        unique_comm_periods(end+1,:) = [comm_event.time, comm_event.time + comm_event.duration];
    end
end

% Výpočet skutečného vytížení frekvence (s překryvy)
frequency_occupancy_percent = 0;
if ~isempty(unique_comm_periods)
    % Seřadit a sloučit překrývající se periody
    [~, idx] = sort(unique_comm_periods(:,1));
    sorted_periods = unique_comm_periods(idx, :);
    
    merged_periods = sorted_periods(1,:);
    for i = 2:size(sorted_periods, 1)
        if sorted_periods(i,1) <= merged_periods(end,2)
            merged_periods(end,2) = max(merged_periods(end,2), sorted_periods(i,2));
        else
            merged_periods(end+1,:) = sorted_periods(i,:);
        end
    end
    
    actual_comm_time = sum(merged_periods(:,2) - merged_periods(:,1));
    frequency_occupancy_percent = (actual_comm_time / SIM_DURATION_S) * 100;
end

% PRŮMĚRNÁ DOBA LETOUNU NA FREKVENCI
total_time_on_freq = 0;
aircraft_count = 0;

if ~isempty(scheduled_flights)
    for i = 1:length(scheduled_flights)
        flight = scheduled_flights(i);
        if strcmp(flight.type, 'arrival')
            time_on_freq = flight.landing_time - flight.entry_time;
        else % departure
            time_on_freq = flight.exit_time - flight.takeoff_time;
        end
        total_time_on_freq = total_time_on_freq + time_on_freq;
        aircraft_count = aircraft_count + 1;
    end
end

avg_time_on_frequency_s = 0;
if aircraft_count > 0
    avg_time_on_frequency_s = total_time_on_freq / aircraft_count;
end

% KORELACE WORKLOAD VS POČET LETOUNŮ
correlation_value = NaN;
regression_slope = NaN;
regression_intercept = NaN;

if ~isempty(workload_data_for_graph) && ~isempty(active_aircraft_count_per_interval)
    min_length = min(length(workload_data_for_graph), length(active_aircraft_count_per_interval));
    workload_trimmed = workload_data_for_graph(1:min_length);
    aircraft_trimmed = active_aircraft_count_per_interval(1:min_length);
    
    if length(workload_trimmed) > 1 && std(workload_trimmed) > 0 && std(aircraft_trimmed) > 0
        correlation_matrix = corrcoef(workload_trimmed, aircraft_trimmed);
        correlation_value = correlation_matrix(1,2);
        
        % Regresní analýza
        p = polyfit(aircraft_trimmed, workload_trimmed, 1);
        regression_slope = p(1);
        regression_intercept = p(2);
    end
end

% EFEKTIVITA PLÁNOVÁNÍ
total_requests = FIXED_NUM_AIRCRAFT + FIXED_NUM_DEPARTURES;
rejected_count = length(rejected_flights_info);
rejection_rate_percent = (rejected_count / total_requests) * 100;

% PRŮMĚRNÉ ZPOŽDĚNÍ
total_delay = 0;
delay_count = 0;
all_delays = [];

if ~isempty(scheduled_flights)
    for i = 1:length(scheduled_flights)
        flight = scheduled_flights(i);
        delay_match = regexp(flight.id, '_d(\d+)_', 'tokens');
        if ~isempty(delay_match)
            delay = str2double(delay_match{1}{1});
            total_delay = total_delay + delay;
            all_delays(end+1) = delay;
            delay_count = delay_count + 1;
        end
    end
end

avg_delay_s = 0;
max_delay_s = 0;
std_delay_s = 0;
if delay_count > 0
    avg_delay_s = total_delay / delay_count;
    max_delay_s = max(all_delays);
    std_delay_s = std(all_delays);
end

% VYUŽITÍ KAPACITY
actual_movements = length(scheduled_flights);
movements_per_hour = actual_movements / (SIM_DURATION_S / 3600);
capacity_utilization_percent = (movements_per_hour / THEORETICAL_HOURLY_CAPACITY) * 100;

% ANALÝZA WORKLOADU
workload_mean = mean(workload_data_for_graph);
workload_max = max(workload_data_for_graph);
workload_min = min(workload_data_for_graph);
workload_std = std(workload_data_for_graph);

time_low_percent = sum(workload_data_for_graph < LOW_WORKLOAD_THRESHOLD) / length(workload_data_for_graph) * 100;
time_medium_percent = sum(workload_data_for_graph >= MEDIUM_WORKLOAD_THRESHOLD & workload_data_for_graph < HIGH_WORKLOAD_THRESHOLD) / length(workload_data_for_graph) * 100;
time_high_percent = sum(workload_data_for_graph >= HIGH_WORKLOAD_THRESHOLD & workload_data_for_graph < OVERLOAD_THRESHOLD) / length(workload_data_for_graph) * 100;
time_overload_percent = sum(workload_data_for_graph >= OVERLOAD_THRESHOLD) / length(workload_data_for_graph) * 100;

% ZOBRAZENÍ STATISTIK
fprintf('\n========== ROZŠÍŘENÉ STATISTICKÉ VÝSLEDKY ==========\n');

fprintf('\n--- KOMUNIKAČNÍ STATISTIKY ---\n');
fprintf('Celkový čas komunikace: %.1f s (%.2f min)\n', total_comm_time_s, total_comm_time_s/60);
fprintf('Vytížení frekvence: %.1f%%\n', frequency_occupancy_percent);
fprintf('Průměrná doba letounu na frekvenci: %.1f s (%.2f min)\n', avg_time_on_frequency_s, avg_time_on_frequency_s/60);

fprintf('\n--- KORELACE ZÁTĚŽE ---\n');
if ~isnan(correlation_value)
    fprintf('Korelace workload vs. počet letounů: %.3f\n', correlation_value);
    fprintf('Nárůst workloadu na jedno letadlo: %.2f bodů\n', regression_slope);
    fprintf('Základní workload (0 letadel): %.2f bodů\n', regression_intercept);
else
    fprintf('Korelace workload vs. počet letounů: Nedostatek dat\n');
end

fprintf('\n--- EFEKTIVITA PLÁNOVÁNÍ ---\n');
fprintf('Celkový počet požadavků: %d\n', total_requests);
fprintf('Přijato: %d letů\n', length(scheduled_flights));
fprintf('Odmítnuto: %d letů (%.1f%%)\n', rejected_count, rejection_rate_percent);
fprintf('Průměrné zpoždění: %.1f s (%.2f min)\n', avg_delay_s, avg_delay_s/60);
fprintf('Maximální zpoždění: %.1f s (%.2f min)\n', max_delay_s, max_delay_s/60);
fprintf('Směrodatná odchylka zpoždění: %.1f s\n', std_delay_s);

fprintf('\n--- VYUŽITÍ KAPACITY ---\n');
fprintf('Skutečný počet pohybů: %d\n', actual_movements);
fprintf('Pohybů za hodinu: %.2f\n', movements_per_hour);
fprintf('Teoretická kapacita: %.0f pohybů/hod\n', THEORETICAL_HOURLY_CAPACITY);
fprintf('Využití kapacity: %.1f%%\n', capacity_utilization_percent);

fprintf('\n--- ANALÝZA ZÁTĚŽE ---\n');
fprintf('Průměrná zátěž: %.1f bodů\n', workload_mean);
fprintf('Maximální zátěž: %.1f bodů\n', workload_max);
fprintf('Minimální zátěž: %.1f bodů\n', workload_min);
fprintf('Směrodatná odchylka: %.1f bodů\n', workload_std);
fprintf('Čas v nízkém zatížení (<60): %.1f%%\n', time_low_percent);
fprintf('Čas ve středním zatížení (90-162): %.1f%%\n', time_medium_percent);
fprintf('Čas ve vysokém zatížení (162-210): %.1f%%\n', time_high_percent);
fprintf('Čas v přetížení (>210): %.1f%%\n', time_overload_percent);

fprintf('\n=================================================\n');

% ----------------- GRAFICKÉ VÝSTUPY - ZJEDNODUŠENÝ GRAF -------------------
if ~isempty(workload_data_for_graph) && any(workload_data_for_graph)
    figure('Name', 'Analýza zátěže řídícího - TMA Čáslav', 'Position', [150, 150, 1100, 600]);
    time_minutes_wl = (0:length(workload_data_for_graph)-1) * (WORKLOAD_INTERVAL_S / 60);
    plot(time_minutes_wl, workload_data_for_graph, 'b-', 'LineWidth', 2);
    hold on;
    
    % Vyznačení prioritních slotů
    if ENABLE_PRIORITY_DEPARTURE_SLOTS && ~isempty(PRIORITY_DEPARTURE_TIMES_S)
        for pdt = PRIORITY_DEPARTURE_TIMES_S
            xline(pdt/60, '--r', 'LineWidth', 1.5);
            text(pdt/60 + 0.5, max(workload_data_for_graph)*0.95, 'P', 'Color', 'r', 'FontWeight', 'bold', 'FontSize', 12);
        end
    end
    
    % ZJEDNODUŠENÉ PRAHOVÉ HODNOTY - pouze 3 čáry podle požadavku
    yline(OVERLOAD_THRESHOLD, '--r', ['Přetížení (>', num2str(OVERLOAD_THRESHOLD), ')'], 'Color', 'r', 'LineWidth', 2, 'LabelHorizontalAlignment', 'left','FontSize',12); 
    yline(HIGH_WORKLOAD_THRESHOLD, '--', ['Vysoká zátěž (', num2str(HIGH_WORKLOAD_THRESHOLD), '-', num2str(OVERLOAD_THRESHOLD), ')'], 'Color', [1, 0.7, 0], 'LineWidth', 2, 'LabelHorizontalAlignment', 'left','FontSize',12);
    yline(MEDIUM_WORKLOAD_THRESHOLD, '--g', ['Střední zátěž (', num2str(MEDIUM_WORKLOAD_THRESHOLD), '-', num2str(HIGH_WORKLOAD_THRESHOLD), ')'], 'LineWidth', 2, 'LabelHorizontalAlignment', 'left', 'Color', 'g', 'FontSize',12);

    xlabel('Čas (minuty)');
    ylabel('Zátěž řídícího (jednotky)'); 
    title_str = 'Analýza pracovní zátěže řídícího - TMA Čáslav';
    if ENABLE_PRIORITY_DEPARTURE_SLOTS
        title_str = [title_str, ' (s prioritními sloty)'];
    end
    title(title_str); 
    grid on;
    
    legend_handles = [plot(NaN,NaN,'b-','LineWidth',2), ...
                      line(NaN,NaN, 'LineStyle', '--', 'Color', 'g'), ...
                      line(NaN,NaN, 'LineStyle', '--', 'Color', [1, 0.7, 0]), ...
                      line(NaN,NaN, 'LineStyle', '--', 'Color', 'r')];
    legend_texts = {'Vypočtená zátěž', 'Střední zátěž (90-162)', 'Vysoká zátěž (162-210)', 'Přetížení (>210)'};
    if ENABLE_PRIORITY_DEPARTURE_SLOTS
        legend_handles(end+1) = line(NaN,NaN, 'LineStyle', '--', 'Color', 'r', 'LineWidth', 1.5);
        legend_texts{end+1} = 'Prioritní slot';
    end
    legend(legend_handles, legend_texts, 'Location', 'NorthEast');
    
    max_y_val_for_graph = max([max(workload_data_for_graph) * 1.1, OVERLOAD_THRESHOLD * 1.2, 100]);
    if isempty(max_y_val_for_graph) || isnan(max_y_val_for_graph) || max_y_val_for_graph == 0, max_y_val_for_graph = 250; end
    ylim([0, max_y_val_for_graph]);
    hold off;
end

% Graf korelace workload vs počet letounů
if ~isempty(active_aircraft_count_per_interval) && ~isempty(workload_data_for_graph) && length(active_aircraft_count_per_interval) == length(workload_data_for_graph)
    figure('Name', 'Korelace zátěže a počtu letounů', 'Position', [350, 350, 900, 700]);
    
    % Scatter plot s regresní přímkou
    subplot(2,1,1);
    scatter(active_aircraft_count_per_interval, workload_data_for_graph, 50, 'b', 'filled', 'MarkerEdgeColor', 'k');
    hold on;
    
    % Regresní přímka
    if ~isnan(regression_slope)
        x_fit = 0:0.1:max(active_aircraft_count_per_interval);
        y_fit = regression_slope * x_fit + regression_intercept;
        plot(x_fit, y_fit, 'r-', 'LineWidth', 2);
        
        % Confidence interval (volitelné)
        text(max(active_aircraft_count_per_interval)*0.7, max(workload_data_for_graph)*0.9, ...
            sprintf('y = %.2fx + %.2f\nr = %.3f', regression_slope, regression_intercept, correlation_value), ...
            'FontSize', 12, 'BackgroundColor', 'white');
    end
    
    xlabel('Počet aktivních letounů');
    ylabel('Zátěž řídícího (body)');
    title(sprintf('Korelace workload vs. počet letounů (r = %.3f)', correlation_value));
    grid on;
    legend('Data', 'Regresní přímka', 'Location', 'NorthWest');
    
    % Časový průběh obou veličin
    subplot(2,1,2);
    time_minutes = (0:length(active_aircraft_count_per_interval)-1) * (WORKLOAD_INTERVAL_S / 60);
    
    yyaxis left;
    plot(time_minutes, active_aircraft_count_per_interval, 'b-', 'LineWidth', 2);
    ylabel('Počet aktivních letounů');
    ylim([0, max(active_aircraft_count_per_interval)*1.2]);
    
    yyaxis right;
    plot(time_minutes, workload_data_for_graph, 'r-', 'LineWidth', 2);
    ylabel('Zátěž řídícího (body)');
    ylim([0, max(workload_data_for_graph)*1.2]);
    
    % Vyznačení prioritních slotů
    if ENABLE_PRIORITY_DEPARTURE_SLOTS && ~isempty(PRIORITY_DEPARTURE_TIMES_S)
        for pdt = PRIORITY_DEPARTURE_TIMES_S
            xline(pdt/60, '--k', 'LineWidth', 1);
        end
    end
    
    xlabel('Čas (minuty)');
    title('Časový průběh počtu letounů a zátěže');
    grid on;
    legend('Počet letounů', 'Zátěž', 'Location', 'NorthEast');
end

% Graf komunikační zátěže
figure('Name', 'Analýza komunikační zátěže', 'Position', [450, 450, 800, 600]);

% Časová osa komunikací
if ~isempty(aircraft_comm_events_for_dyn_sim)
    subplot(2,1,1);
    hold on;
    
    % Vykreslení komunikačních událostí jako horizontální čáry
    for i = 1:length(aircraft_comm_events_for_dyn_sim)
        event = aircraft_comm_events_for_dyn_sim(i);
        start_time = event.time / 60; % převod na minuty
        end_time = (event.time + event.duration) / 60;
        
        % Různé barvy pro různé typy komunikací
        if event.points > 30
            color = 'r'; % vysoká zátěž
        elseif event.points > 20
            color = [1 0.5 0]; % střední zátěž
        else
            color = 'g'; % nízká zátěž
        end
        
        plot([start_time, end_time], [i, i], '-', 'Color', color, 'LineWidth', 3);
    end
    
    xlabel('Čas (minuty)');
    ylabel('Komunikační událost');
    title('Časová osa komunikačních událostí');
    xlim([0, SIM_DURATION_S/60]);
    ylim([0, length(aircraft_comm_events_for_dyn_sim)+1]);
    grid on;
    
    % Histogram vytížení
    subplot(2,1,2);
    time_bins = 0:1:SIM_DURATION_S/60; % minutové intervaly
    comm_load = zeros(size(time_bins));
    
    for i = 1:length(time_bins)-1
        bin_start = time_bins(i) * 60;
        bin_end = time_bins(i+1) * 60;
        
        for j = 1:length(aircraft_comm_events_for_dyn_sim)
            event = aircraft_comm_events_for_dyn_sim(j);
            overlap_start = max(event.time, bin_start);
            overlap_end = min(event.time + event.duration, bin_end);
            
            if overlap_end > overlap_start
                comm_load(i) = comm_load(i) + (overlap_end - overlap_start);
            end
        end
    end
    
    bar(time_bins(1:end-1), comm_load(1:end-1) / 60 * 100, 'FaceColor', [0.3 0.5 0.8]);
    xlabel('Čas (minuty)');
    ylabel('Vytížení frekvence (%)');
    title('Vytížení komunikační frekvence v čase');
    ylim([0, 100]);
    grid on;
end

% Uložení výsledků pro pozdější analýzu
disp(' ');
disp('--- Ukládání výsledků ---');
results = struct();
results.scheduled_flights = scheduled_flights;
results.rejected_flights = rejected_flights_info;
results.workload_profile = workload_data_for_graph;
results.active_aircraft_profile = active_aircraft_count_per_interval;
results.priority_slots_enabled = ENABLE_PRIORITY_DEPARTURE_SLOTS;
results.priority_departure_times = PRIORITY_DEPARTURE_TIMES_S;
results.priority_departures_count = priority_departures_count;
results.regular_departures_count = regular_departures_count;
results.statistics = struct();
results.statistics.comm_total_time_s = total_comm_time_s;
results.statistics.frequency_occupancy_percent = frequency_occupancy_percent;
results.statistics.avg_time_on_frequency_s = avg_time_on_frequency_s;
results.statistics.correlation_workload_aircraft = correlation_value;
results.statistics.regression_slope = regression_slope;
results.statistics.rejection_rate_percent = rejection_rate_percent;
results.statistics.avg_delay_s = avg_delay_s;
results.statistics.movements_per_hour = movements_per_hour;
results.statistics.capacity_utilization_percent = capacity_utilization_percent;
results.statistics.workload_mean = workload_mean;
results.statistics.workload_max = workload_max;
results.statistics.time_overload_percent = time_overload_percent;

save_filename = sprintf('TMA_results_priority_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
save(save_filename, 'results');
disp(['Výsledky uloženy do souboru: ', save_filename]);

disp(' ');
disp('Simulace TMA Čáslav s prioritními sloty úspěšně dokončena!');

% ----------------- POMOCNÉ FUNKCE -------------------

% --- NOVÁ FUNKCE PRO PLÁNOVÁNÍ S PRIORITNÍMI SLOTY ---
function [planned_flights, rejected_flights_log, workload_profile, runway_ops_log] = ...
    plan_flights_with_priority_slots(orig_arrivals_t, orig_departures_t, ...
    arr_routes, dep_route, arr_route_prob, sim_dur_s, wl_interval_s, ...
    max_wl_points, base_wl_points, comm_params, min_sep_air_nm, ...
    rwy_occ_land_s, arr_arr_int_s, dep_arr_int_s, time_to_cv314_s, ...
    priority_departure_times_s, priority_protection_before_s, priority_protection_after_s)

    % Inicializace
    planned_flights = struct('id', {}, 'type', {}, 'route_index', {}, ...
                           'entry_time', {}, 'faf_time', {}, 'landing_time', {}, ...
                           'takeoff_time', {}, 'cf314_time', {}, 'eruso_time', {}, ...
                           'exit_time', {}, 'actual_operation_time', {}, ...
                           'is_priority', {}, 'taken_off', {}, 'landed', {});
    rejected_flights_log = [];
    runway_ops_log = struct('time', {}, 'type', {}, 'flight_id', {}, 'detail_time', {});
    
    num_wl_intervals = ceil(sim_dur_s / wl_interval_s);
    workload_profile = ones(1, num_wl_intervals) * base_wl_points;
    
    scheduled_comm_workload_points = struct('time', {}, 'points', {}, 'flight_id', {}, 'duration', {});

    % === FÁZE 1: REZERVACE PRIORITNÍCH ODLETOVÝCH SLOTŮ ===
    disp('Fáze 1: Rezervace prioritních odletových slotů...');
    
    priority_slots_used = 0;
    for p_idx = 1:length(priority_departure_times_s)
        if p_idx > length(orig_departures_t)
            break; % Nemáme dost odletů pro všechny prioritní sloty
        end
        
        priority_takeoff_time = priority_departure_times_s(p_idx);
        
        % Vytvoření prioritního odletu
        priority_flight = struct();
        priority_flight.id = ['DEP_PRIORITY_', num2str(p_idx)];
        priority_flight.type = 'departure';
        priority_flight.route_index = NaN;
        priority_flight.takeoff_time = priority_takeoff_time;
        priority_flight.cf314_time = priority_takeoff_time + time_to_cv314_s;
        priority_flight.eruso_time = priority_flight.cf314_time + dep_route.segment_time(2);
        priority_flight.exit_time = priority_flight.eruso_time;
        priority_flight.actual_operation_time = priority_flight.takeoff_time;
        priority_flight.is_priority = true;
        priority_flight.entry_time = NaN;
        priority_flight.faf_time = NaN;
        priority_flight.landing_time = NaN;
        priority_flight.taken_off = false;
        priority_flight.landed = false;
        
        % Kontrola, zda se vejde do simulace
        if priority_flight.exit_time > sim_dur_s
            disp(['Prioritní slot v čase ', num2str(priority_takeoff_time), 's přesahuje dobu simulace - přeskakuji']);
            continue;
        end
        
        % Vytvoření komunikačních událostí
        temp_comm_events = struct('time', {}, 'points', {}, 'flight_id', {}, 'duration', {});
        temp_comm_events(end+1) = struct('time', priority_flight.takeoff_time + comm_params.tower_radar_coordination.timing_offset_s, ...
            'points', comm_params.tower_radar_coordination.controller_workload_points, ...
            'flight_id', priority_flight.id, 'duration', comm_params.tower_radar_coordination.duration_total_transaction_s);
        temp_comm_events(end+1) = struct('time', priority_flight.cf314_time + comm_params.post_takeoff_contact.timing_offset_s, ...
            'points', comm_params.post_takeoff_contact.controller_workload_points, ...
            'flight_id', priority_flight.id, 'duration', comm_params.post_takeoff_contact.duration_total_transaction_s);
        temp_comm_events(end+1) = struct('time', priority_flight.eruso_time + comm_params.final_contact.timing_offset_s, ...
            'points', comm_params.final_contact.controller_workload_points, ...
            'flight_id', priority_flight.id, 'duration', comm_params.final_contact.duration_total_transaction_s);
        
        % Kontrola workloadu
        [workload_ok, temp_workload_profile] = check_complex_workload_impact(...
            priority_flight, temp_comm_events, workload_profile, scheduled_comm_workload_points, ...
            num_wl_intervals, wl_interval_s, base_wl_points, max_wl_points, ...
            planned_flights, arr_routes, dep_route, time_to_cv314_s);
        
        if workload_ok
            % Přidat prioritní odlet
            planned_flights(end+1) = priority_flight;
            workload_profile = temp_workload_profile;
            scheduled_comm_workload_points = [scheduled_comm_workload_points, temp_comm_events];
            
            % Zaznamenat využití dráhy
            new_op_log = struct('time', priority_flight.takeoff_time, ...
                              'type', 'departure', ...
                              'flight_id', priority_flight.id, ...
                              'detail_time', priority_flight.cf314_time);
            runway_ops_log = [runway_ops_log, new_op_log];
            
            priority_slots_used = priority_slots_used + 1;
            disp(['  Rezervován prioritní slot ', num2str(p_idx), ' v čase ', num2str(priority_takeoff_time/60, '%.1f'), ' min']);
        else
            warning(['KRITICKÉ: Nelze rezervovat prioritní slot v čase ', num2str(priority_takeoff_time), 's kvůli workloadu!']);
            % V striktní variantě považujeme toto za kritickou chybu
        end
    end
    
    disp(['  Celkem rezervováno ', num2str(priority_slots_used), ' prioritních slotů']);
    
    % Seřadit runway_ops_log
    if ~isempty(runway_ops_log)
        [~, sort_idx] = sort([runway_ops_log.time]);
        runway_ops_log = runway_ops_log(sort_idx);
    end
    
    % === FÁZE 2: PLÁNOVÁNÍ PŘÍLETŮ ===
    disp('Fáze 2: Plánování příletů s respektováním prioritních slotů...');
    
    % Vytvoření zakázaných zón kolem prioritních odletů
    forbidden_zones = [];
    for i = 1:length(planned_flights)
        if planned_flights(i).is_priority
            zone_start = planned_flights(i).takeoff_time - priority_protection_before_s;
            zone_end = planned_flights(i).takeoff_time + priority_protection_after_s;
            forbidden_zones(end+1,:) = [zone_start, zone_end];
        end
    end
    
    % Plánování příletů
    for i = 1:length(orig_arrivals_t)
        route_index = select_route(arr_route_prob);
        route = arr_routes{route_index};
        orig_entry_time = orig_arrivals_t(i);
        orig_faf_time = orig_entry_time + sum(route.segment_time(1:2));
        orig_landing_time = orig_entry_time + route.total_time;
        
        if orig_landing_time > sim_dur_s
            continue;
        end
        
        required_delay = 0;
        max_delay = sim_dur_s - orig_landing_time;
        flight_planned_successfully = false;
        max_iterations = 500;
        iteration_count = 0;
        
        while ~flight_planned_successfully && required_delay <= max_delay && iteration_count < max_iterations
            iteration_count = iteration_count + 1;
            
            test_entry_time = orig_entry_time + required_delay;
            test_landing_time = orig_landing_time + required_delay;
            
            % Kontrola zakázaných zón
            in_forbidden_zone = false;
            for z = 1:size(forbidden_zones, 1)
                if test_landing_time >= forbidden_zones(z,1) && test_landing_time <= forbidden_zones(z,2)
                    in_forbidden_zone = true;
                    required_delay = forbidden_zones(z,2) - orig_landing_time + 5;
                    break;
                end
            end
            
            if in_forbidden_zone
                continue;
            end
            
            % Vytvoření kandidáta na přílet
            current_flight_candidate = struct();
            current_flight_candidate.id = ['ARR_', num2str(i), '_d', num2str(required_delay), '_'];
            current_flight_candidate.type = 'arrival';
            current_flight_candidate.route_index = route_index;
            current_flight_candidate.entry_time = test_entry_time;
            current_flight_candidate.faf_time = test_entry_time + sum(route.segment_time(1:2));
            current_flight_candidate.landing_time = test_landing_time;
            current_flight_candidate.actual_operation_time = test_landing_time;
            current_flight_candidate.is_priority = false;
            current_flight_candidate.takeoff_time = NaN;
            current_flight_candidate.cf314_time = NaN;
            current_flight_candidate.eruso_time = NaN;
            current_flight_candidate.exit_time = NaN;
            current_flight_candidate.taken_off = false;
            current_flight_candidate.landed = false;
            
            % Kontroly separace
            runway_sep_ok = check_runway_separation_rules_v3(current_flight_candidate, runway_ops_log, ...
                rwy_occ_land_s, arr_arr_int_s, dep_arr_int_s, time_to_cv314_s);
            if ~runway_sep_ok
                required_delay = required_delay + 15;
                continue;
            end
            
            air_sep_ok = true;
            if ~isempty(planned_flights)
                air_sep_ok = check_all_air_separations_v3(current_flight_candidate, planned_flights, ...
                    arr_routes, dep_route, min_sep_air_nm, time_to_cv314_s);
            end
            if ~air_sep_ok
                required_delay = required_delay + 15;
                continue;
            end
            
            % Komunikační události
            temp_comm_events_arr = struct('time', {}, 'points', {}, 'flight_id', {}, 'duration', {});
            temp_comm_events_arr(end+1) = struct('time', current_flight_candidate.entry_time + comm_params.pre_approach_contact.timing_offset_s, ...
                'points', comm_params.pre_approach_contact.controller_workload_points, ...
                'flight_id', current_flight_candidate.id, ...
                'duration', comm_params.pre_approach_contact.duration_total_transaction_s);
            temp_comm_events_arr(end+1) = struct('time', current_flight_candidate.faf_time + comm_params.handoff_to_tower.timing_offset_s, ...
                'points', comm_params.handoff_to_tower.controller_workload_points, ...
                'flight_id', current_flight_candidate.id, ...
                'duration', comm_params.handoff_to_tower.duration_total_transaction_s);
            
            % Kontrola workloadu
            [workload_ok, temp_workload_profile] = check_complex_workload_impact(...
                current_flight_candidate, temp_comm_events_arr, workload_profile, scheduled_comm_workload_points, ...
                num_wl_intervals, wl_interval_s, base_wl_points, max_wl_points, ...
                planned_flights, arr_routes, dep_route, time_to_cv314_s);
            
            if workload_ok
                planned_flights(end+1) = current_flight_candidate;
                workload_profile = temp_workload_profile;
                scheduled_comm_workload_points = [scheduled_comm_workload_points, temp_comm_events_arr];
                
                new_op_log = struct('time', current_flight_candidate.landing_time, ...
                                  'type', 'arrival', ...
                                  'flight_id', current_flight_candidate.id, ...
                                  'detail_time', current_flight_candidate.landing_time);
                runway_ops_log = [runway_ops_log, new_op_log];
                
                if ~isempty(runway_ops_log)
                    [~, sort_idx] = sort([runway_ops_log.time]);
                    runway_ops_log = runway_ops_log(sort_idx);
                end
                
                flight_planned_successfully = true;
            else
                required_delay = required_delay + 30;
            end
        end
        
        if ~flight_planned_successfully
            rejected_flights_log(end+1).id = ['ARR_', num2str(i)];
            rejected_flights_log(end).type = 'arrival';
            rejected_flights_log(end).orig_time = orig_arrivals_t(i);
            rejected_flights_log(end).reason = 'Nelze naplánovat kvůli prioritním slotům, separacím nebo workloadu';
        end
    end
    
    % === FÁZE 3: PLÁNOVÁNÍ ZBÝVAJÍCÍCH ODLETŮ ===
    disp('Fáze 3: Plánování běžných odletů do volných míst...');
    
    % Počet již naplánovaných prioritních odletů
    remaining_departures_start_idx = priority_slots_used + 1;
    
    for i = remaining_departures_start_idx:length(orig_departures_t)
        orig_takeoff_time = orig_departures_t(i);
        
        required_delay = 0;
        max_delay = sim_dur_s - (orig_takeoff_time + dep_route.total_time);
        flight_planned_successfully = false;
        max_iterations = 500;
        iteration_count = 0;
        
        while ~flight_planned_successfully && required_delay <= max_delay && iteration_count < max_iterations
            iteration_count = iteration_count + 1;
            
            test_takeoff_time = orig_takeoff_time + required_delay;
            
            % Kontrola, zda nekoliduje s prioritními sloty
            conflicts_with_priority = false;
            for z = 1:size(forbidden_zones, 1)
                if test_takeoff_time >= forbidden_zones(z,1) && test_takeoff_time <= forbidden_zones(z,2)
                    conflicts_with_priority = true;
                    required_delay = forbidden_zones(z,2) - orig_takeoff_time + 5;
                    break;
                end
            end
            
            if conflicts_with_priority
                continue;
            end
            
            % Vytvoření kandidáta na běžný odlet
            current_flight_candidate = struct();
            current_flight_candidate.id = ['DEP_', num2str(i), '_d', num2str(required_delay), '_'];
            current_flight_candidate.type = 'departure';
            current_flight_candidate.route_index = NaN;
            current_flight_candidate.takeoff_time = test_takeoff_time;
            current_flight_candidate.cf314_time = test_takeoff_time + time_to_cv314_s;
            current_flight_candidate.eruso_time = current_flight_candidate.cf314_time + dep_route.segment_time(2);
            current_flight_candidate.exit_time = current_flight_candidate.eruso_time;
            current_flight_candidate.actual_operation_time = test_takeoff_time;
            current_flight_candidate.is_priority = false;
            current_flight_candidate.entry_time = NaN;
            current_flight_candidate.faf_time = NaN;
            current_flight_candidate.landing_time = NaN;
            current_flight_candidate.taken_off = false;
            current_flight_candidate.landed = false;
            
            if current_flight_candidate.exit_time > sim_dur_s
                break; % Přesahuje simulaci
            end
            
            % Kontroly separace
            runway_sep_ok = check_runway_separation_rules_v3(current_flight_candidate, runway_ops_log, ...
                rwy_occ_land_s, arr_arr_int_s, dep_arr_int_s, time_to_cv314_s);
            if ~runway_sep_ok
                required_delay = required_delay + 15;
                continue;
            end
            
            air_sep_ok = true;
            if ~isempty(planned_flights)
                air_sep_ok = check_all_air_separations_v3(current_flight_candidate, planned_flights, ...
                    arr_routes, dep_route, min_sep_air_nm, time_to_cv314_s);
            end
            if ~air_sep_ok
                required_delay = required_delay + 15;
                continue;
            end
            
            % Komunikační události
            temp_comm_events_dep = struct('time', {}, 'points', {}, 'flight_id', {}, 'duration', {});
            temp_comm_events_dep(end+1) = struct('time', current_flight_candidate.takeoff_time + comm_params.tower_radar_coordination.timing_offset_s, ...
                'points', comm_params.tower_radar_coordination.controller_workload_points, ...
                'flight_id', current_flight_candidate.id, ...
                'duration', comm_params.tower_radar_coordination.duration_total_transaction_s);
            temp_comm_events_dep(end+1) = struct('time', current_flight_candidate.cf314_time + comm_params.post_takeoff_contact.timing_offset_s, ...
                'points', comm_params.post_takeoff_contact.controller_workload_points, ...
                'flight_id', current_flight_candidate.id, ...
                'duration', comm_params.post_takeoff_contact.duration_total_transaction_s);
            temp_comm_events_dep(end+1) = struct('time', current_flight_candidate.eruso_time + comm_params.final_contact.timing_offset_s, ...
                'points', comm_params.final_contact.controller_workload_points, ...
                'flight_id', current_flight_candidate.id, ...
                'duration', comm_params.final_contact.duration_total_transaction_s);
            
            % Kontrola workloadu
            [workload_ok, temp_workload_profile] = check_complex_workload_impact(...
                current_flight_candidate, temp_comm_events_dep, workload_profile, scheduled_comm_workload_points, ...
                num_wl_intervals, wl_interval_s, base_wl_points, max_wl_points, ...
                planned_flights, arr_routes, dep_route, time_to_cv314_s);
            
            if workload_ok
                planned_flights(end+1) = current_flight_candidate;
                workload_profile = temp_workload_profile;
                scheduled_comm_workload_points = [scheduled_comm_workload_points, temp_comm_events_dep];
                
                new_op_log = struct('time', current_flight_candidate.takeoff_time, ...
                                  'type', 'departure', ...
                                  'flight_id', current_flight_candidate.id, ...
                                  'detail_time', current_flight_candidate.cf314_time);
                runway_ops_log = [runway_ops_log, new_op_log];
                
                if ~isempty(runway_ops_log)
                    [~, sort_idx] = sort([runway_ops_log.time]);
                    runway_ops_log = runway_ops_log(sort_idx);
                end
                
                flight_planned_successfully = true;
            else
                required_delay = required_delay + 30;
            end
        end
        
        if ~flight_planned_successfully
            rejected_flights_log(end+1).id = ['DEP_', num2str(i)];
            rejected_flights_log(end).type = 'departure';
            rejected_flights_log(end).orig_time = orig_departures_t(i);
            rejected_flights_log(end).reason = 'Nelze naplánovat běžný odlet - konflikt s prioritními sloty nebo nedostatek kapacity';
        end
    end
    
    % Finální seřazení
    if ~isempty(planned_flights)
        [~, sort_final_idx] = sort([planned_flights.actual_operation_time]);
        planned_flights = planned_flights(sort_final_idx);
    end
    
    disp(['  Naplánováno celkem: ', num2str(length(planned_flights)), ' letů']);
end

% --- Ostatní pomocné funkce zůstávají beze změny ---
% [Všechny ostatní funkce z původního S1 zůstávají stejné]

% --- Hlavní funkce komplexního modelu zátěže (upravené parametry) ---
function controller_workload = calculate_complete_workload_v4(active_aircraft_positions, comm_events, current_minute, conflicts_detected, comm_backlog, base_workload, interval_duration_s)
    
    total_active_aircraft = size(active_aircraft_positions, 1);
    
    % 1. Základní zátěž
    % base_workload je předán jako parametr
    
    % 2. Monitoring zátěž - původní hodnoty
    basic_tracking = total_active_aircraft * 2; % Vráceno na původní hodnotu
    
    num_aircraft_pairs = 0;
    if total_active_aircraft >= 2
        num_aircraft_pairs = total_active_aircraft * (total_active_aircraft - 1) / 2;
    end
    separation_monitoring = num_aircraft_pairs * 0.5; % Vráceno na původní hodnotu
    
    prediction_base = total_active_aircraft * 1; % Vráceno na původní hodnotu
    if total_active_aircraft > 3
        complexity_factor = (total_active_aircraft - 3) * 0.5; % Vráceno na původní hodnotu
        prediction_workload = prediction_base * (1 + complexity_factor);
    else
        prediction_workload = prediction_base;
    end
    
    monitoring_workload = basic_tracking + separation_monitoring + prediction_workload;
    
    % 3. Komunikační zátěž (používá již snížené hodnoty z comm_events)
    comm_workload_val = 0;
    minute_start = (current_minute - 1) * interval_duration_s;
    minute_end = current_minute * interval_duration_s;
    
    for e_idx = 1:length(comm_events)
        if comm_events(e_idx).time >= minute_start && comm_events(e_idx).time < minute_end
            % Převod na zátěž za minutu podle původního modelu
            comm_workload_val = comm_workload_val + comm_events(e_idx).points * (comm_events(e_idx).duration / interval_duration_s);
        end
    end
    
    % 4. Konfliktní zátěž na základě vzdáleností (původní hodnoty)
    conflict_workload_val = 0;
    for ac1 = 1:size(active_aircraft_positions, 1)
        for ac2 = ac1+1:size(active_aircraft_positions, 1)
            distance = haversine_distance(active_aircraft_positions(ac1,1), active_aircraft_positions(ac1,2), ...
                                        active_aircraft_positions(ac2,1), active_aircraft_positions(ac2,2));
            if distance < 10 && distance >= 7
                conflict_workload_val = conflict_workload_val + 5; % Vráceno na původní hodnotu
            elseif distance < 7 && distance >= 5
                conflict_workload_val = conflict_workload_val + 10; % Vráceno na původní hodnotu
            elseif distance < 5
                conflict_workload_val = conflict_workload_val + 20; % Vráceno na původní hodnotu
            end
        end
    end
    
    % 5. Kognitivní zátěž (upravený multiplikátor na 10%)
    cognitive_load = 10; % Vráceno na původní hodnotu
    if total_active_aircraft > 4
        cognitive_multiplier = 1 + (total_active_aircraft - 4) * 0.10; % Změněno z 0.15 na 0.10
        cognitive_load = cognitive_load * cognitive_multiplier;
    end
    
    % Stresové faktory (původní hodnoty)
    if conflicts_detected
        cognitive_load = cognitive_load + 20; % Vráceno na původní hodnotu
    end
    if comm_backlog > 2
        cognitive_load = cognitive_load + 10; % Vráceno na původní hodnotu
    end
    
    % Celková zátěž
    controller_workload = base_workload + monitoring_workload + comm_workload_val + conflict_workload_val + cognitive_load;
end

% --- Upravená plánovací funkce s komplexním modelem ---
function [planned_flights, rejected_flights_log, workload_profile, runway_ops_log] = ...
    plan_flights_with_complex_workload_control(orig_arrivals_t, orig_departures_t, ...
    arr_routes, dep_route, arr_route_prob, sim_dur_s, wl_interval_s, ...
    max_wl_points, base_wl_points, comm_params, min_sep_air_nm, ...
    rwy_occ_land_s, arr_arr_int_s, dep_arr_int_s, time_to_cv314_s)

    all_requests = [];
    for i = 1:length(orig_arrivals_t)
        all_requests(end+1).orig_time = orig_arrivals_t(i);
        all_requests(end).type = 'arrival';
        all_requests(end).id = ['ARR_orig_', num2str(i)];
    end
    for i = 1:length(orig_departures_t)
        all_requests(end+1).orig_time = orig_departures_t(i);
        all_requests(end).type = 'departure';
        all_requests(end).id = ['DEP_orig_', num2str(i)];
    end

    if isempty(all_requests)
        planned_flights = []; rejected_flights_log = []; workload_profile = []; runway_ops_log = [];
        return;
    end

    [~, sort_idx] = sort([all_requests.orig_time]); 
    sorted_requests = all_requests(sort_idx);

    planned_flights = struct('id', {}, 'type', {}, 'route_index', {}, ...
                             'entry_time', {}, 'faf_time', {}, 'landing_time', {}, ...
                             'takeoff_time', {}, 'cf314_time', {}, 'eruso_time', {}, 'exit_time', {}, ...
                             'actual_operation_time', {}, 'taken_off', {}, 'landed', {}); 
    rejected_flights_log = [];
    runway_ops_log = struct('time', {}, 'type', {}, 'flight_id', {}, 'detail_time', {}); 
    
    num_wl_intervals = ceil(sim_dur_s / wl_interval_s);
    workload_profile = ones(1, num_wl_intervals) * base_wl_points; 
    
    scheduled_comm_workload_points = struct('time', {}, 'points', {}, 'flight_id', {}, 'duration', {}); 

    MAX_PLANNING_DELAY_S = 600; 
    TIME_STEP_PLANNING_S = 15;               % Jemnější krok 15s

    for req_idx = 1:length(sorted_requests)
        request = sorted_requests(req_idx);
        flight_planned_successfully = false;
        
        for delay_s = 0:TIME_STEP_PLANNING_S:MAX_PLANNING_DELAY_S
            current_flight_candidate = struct(); 
            current_flight_candidate.id = strrep(request.id, '_orig_', ['_d', num2str(delay_s),'_']);
            current_flight_candidate.type = request.type;
            
            temp_comm_events_for_this_flight = struct('time', {}, 'points', {}, 'flight_id', {}, 'duration', {}); 

            if strcmp(request.type, 'arrival')
                current_flight_candidate.route_index = select_route(arr_route_prob);
                current_route_details = arr_routes{current_flight_candidate.route_index};
                current_flight_candidate.entry_time = request.orig_time + delay_s;
                current_flight_candidate.faf_time = current_flight_candidate.entry_time + sum(current_route_details.segment_time(1:2));
                current_flight_candidate.landing_time = current_flight_candidate.entry_time + current_route_details.total_time;
                current_flight_candidate.actual_operation_time = current_flight_candidate.landing_time; 
                current_flight_candidate.takeoff_time = NaN; current_flight_candidate.cf314_time = NaN;
                current_flight_candidate.eruso_time = NaN; current_flight_candidate.exit_time = NaN;
                current_flight_candidate.taken_off = false; current_flight_candidate.landed = false;

                if current_flight_candidate.landing_time > sim_dur_s, continue; end 

                temp_comm_events_for_this_flight(end+1) = struct('time', current_flight_candidate.entry_time + comm_params.pre_approach_contact.timing_offset_s, 'points', comm_params.pre_approach_contact.controller_workload_points, 'flight_id', current_flight_candidate.id, 'duration', comm_params.pre_approach_contact.duration_total_transaction_s);
                temp_comm_events_for_this_flight(end+1) = struct('time', current_flight_candidate.faf_time + comm_params.handoff_to_tower.timing_offset_s, 'points', comm_params.handoff_to_tower.controller_workload_points, 'flight_id', current_flight_candidate.id, 'duration', comm_params.handoff_to_tower.duration_total_transaction_s);

            else % Departure
                current_flight_candidate.takeoff_time = request.orig_time + delay_s;
                current_flight_candidate.cf314_time = current_flight_candidate.takeoff_time + time_to_cv314_s;
                current_flight_candidate.eruso_time = current_flight_candidate.cf314_time + dep_route.segment_time(2);
                current_flight_candidate.exit_time = current_flight_candidate.eruso_time;
                current_flight_candidate.actual_operation_time = current_flight_candidate.takeoff_time;
                current_flight_candidate.route_index = NaN; current_flight_candidate.entry_time = NaN;
                current_flight_candidate.faf_time = NaN; current_flight_candidate.landing_time = NaN;
                current_flight_candidate.taken_off = false; current_flight_candidate.landed = false;

                if current_flight_candidate.exit_time > sim_dur_s, continue; end 
                
                temp_comm_events_for_this_flight(end+1) = struct('time', current_flight_candidate.takeoff_time + comm_params.tower_radar_coordination.timing_offset_s, 'points', comm_params.tower_radar_coordination.controller_workload_points, 'flight_id', current_flight_candidate.id, 'duration', comm_params.tower_radar_coordination.duration_total_transaction_s);
                temp_comm_events_for_this_flight(end+1) = struct('time', current_flight_candidate.cf314_time + comm_params.post_takeoff_contact.timing_offset_s, 'points', comm_params.post_takeoff_contact.controller_workload_points, 'flight_id', current_flight_candidate.id, 'duration', comm_params.post_takeoff_contact.duration_total_transaction_s);
                temp_comm_events_for_this_flight(end+1) = struct('time', current_flight_candidate.eruso_time + comm_params.final_contact.timing_offset_s, 'points', comm_params.final_contact.controller_workload_points, 'flight_id', current_flight_candidate.id, 'duration', comm_params.final_contact.duration_total_transaction_s);
            end
            
            runway_sep_ok = check_runway_separation_rules_v3(current_flight_candidate, runway_ops_log, ...
                                                            rwy_occ_land_s, arr_arr_int_s, dep_arr_int_s, time_to_cv314_s);
            if ~runway_sep_ok, continue; end 

            air_sep_ok = true; 
            if ~isempty(planned_flights)
                 air_sep_ok = check_all_air_separations_v3(current_flight_candidate, planned_flights, arr_routes, dep_route, min_sep_air_nm, time_to_cv314_s);
            end
            if ~air_sep_ok, continue; end

            % POUŽITÍ KOMPLEXNÍHO MODELU PRO KONTROLU ZÁTĚŽE
            [workload_ok, temp_workload_profile] = check_complex_workload_impact(...
                                                    current_flight_candidate, temp_comm_events_for_this_flight, ...
                                                    workload_profile, scheduled_comm_workload_points, ...
                                                    num_wl_intervals, wl_interval_s, base_wl_points, max_wl_points, ...
                                                    planned_flights, arr_routes, dep_route, time_to_cv314_s); 
            if ~workload_ok, continue; end
            
            planned_flights(end+1) = current_flight_candidate; 
            workload_profile = temp_workload_profile; 
            scheduled_comm_workload_points = [scheduled_comm_workload_points, temp_comm_events_for_this_flight];
            if ~isempty(scheduled_comm_workload_points) 
                 [~, sort_comm_idx] = sort([scheduled_comm_workload_points.time]);
                 scheduled_comm_workload_points = scheduled_comm_workload_points(sort_comm_idx);
            end

            new_op_log = struct('time', current_flight_candidate.actual_operation_time, 'type', current_flight_candidate.type, 'flight_id', current_flight_candidate.id);
            if strcmp(current_flight_candidate.type, 'arrival')
                new_op_log.detail_time = current_flight_candidate.landing_time; 
            else 
                new_op_log.detail_time = current_flight_candidate.cf314_time; 
            end
            runway_ops_log = [runway_ops_log, new_op_log];
            if ~isempty(runway_ops_log) 
                [~, sort_rwy_idx] = sort([runway_ops_log.time]);
                runway_ops_log = runway_ops_log(sort_rwy_idx);
            end
            
            flight_planned_successfully = true;
            break; 
        end 

        if ~flight_planned_successfully
            rejected_flights_log(end+1).id = request.id;
            rejected_flights_log(end).type = request.type;
            rejected_flights_log(end).orig_time = request.orig_time;
            rejected_flights_log(end).reason = 'Nenalezen slot splňující separace a limit zátěže.';
        end
    end 
    
    if ~isempty(planned_flights)
        [~, sort_final_idx] = sort([planned_flights.actual_operation_time]);
        planned_flights = planned_flights(sort_final_idx);
    end
end

% --- Kontrola zátěže s komplexním modelem ---
function [workload_is_ok, new_workload_profile] = check_complex_workload_impact(...
    candidate_flight, candidate_comm_events, ...
    current_workload_profile, existing_comm_events, ...
    num_intervals, interval_s, base_wl, max_wl, all_currently_planned_flights, ...
    arr_routes, dep_route, time_to_cv314)

    workload_is_ok = true;
    new_workload_profile = current_workload_profile; 

    temp_all_comm_events = [existing_comm_events, candidate_comm_events]; 
    if ~isempty(temp_all_comm_events)
        [~, sort_idx] = sort([temp_all_comm_events.time]);
        temp_all_comm_events = temp_all_comm_events(sort_idx);
    else
        temp_all_comm_events = struct('time', {}, 'points', {}, 'flight_id', {}, 'duration', {}); 
    end
    
    temp_all_planned_flights = [all_currently_planned_flights, candidate_flight];

    for i_interval = 1:num_intervals
        interval_start_t = (i_interval - 1) * interval_s;
        interval_end_t = i_interval * interval_s;
        
        % Získání aktivních letadel v daném intervalu
        active_aircraft_positions = [];
        conflicts_detected = false;
        comm_backlog = 0;
        
        % Projdeme všechny lety a zjistíme které jsou aktivní v tomto intervalu
        for k_fl = 1:length(temp_all_planned_flights)
            fl_obj = temp_all_planned_flights(k_fl);
            fl_s_time = 0; fl_e_time = 0;
            if strcmp(fl_obj.type, 'arrival')
                fl_s_time = fl_obj.entry_time; fl_e_time = fl_obj.landing_time;
            else
                fl_s_time = fl_obj.takeoff_time; fl_e_time = fl_obj.exit_time;
            end
            
            % Pokud je letadlo aktivní v daném intervalu, získáme jeho pozici
            if max(fl_s_time, interval_start_t) < min(fl_e_time, interval_end_t)
                % Získáme pozici uprostřed intervalu
                mid_interval_time = (interval_start_t + interval_end_t) / 2;
                if mid_interval_time >= fl_s_time && mid_interval_time <= fl_e_time
                    pos = get_flight_position_at_time_v3(fl_obj, mid_interval_time, arr_routes, dep_route, time_to_cv314);
                    if ~isempty(pos)
                        active_aircraft_positions(end+1,:) = [pos.lat, pos.lon];
                    end
                end
            end
        end
        
        % Kontrola konfliktů mezi aktivními letadly
        for ac1 = 1:size(active_aircraft_positions, 1)
            for ac2 = ac1+1:size(active_aircraft_positions, 1)
                distance = haversine_distance(active_aircraft_positions(ac1,1), active_aircraft_positions(ac1,2), ...
                                            active_aircraft_positions(ac2,1), active_aircraft_positions(ac2,2));
                if distance < 5 % Konflikt pokud je separace menší než 5 NM
                    conflicts_detected = true;
                end
            end
        end
        
        % Získání komunikačních událostí pro tento interval
        current_comm_events = struct('time', {}, 'points', {}, 'flight_id', {}, 'duration', {}); 
        for k_c_ev = 1:length(temp_all_comm_events)
            c_ev = temp_all_comm_events(k_c_ev);
            if c_ev.time >= interval_start_t && c_ev.time < interval_end_t
                current_comm_events(end+1) = c_ev;
            end
        end
        comm_backlog = max(0, length(current_comm_events) - 2);

        % Výpočet komplexní zátěže pro tento interval
        new_total_wl_for_interval_i = calculate_complete_workload_v4(...
                                         active_aircraft_positions, current_comm_events, ...
                                         i_interval, conflicts_detected, comm_backlog, ...
                                         base_wl, interval_s); 

        if new_total_wl_for_interval_i > max_wl
            workload_is_ok = false;
            return; 
        end
        new_workload_profile(i_interval) = new_total_wl_for_interval_i;
    end
end

% --- Ostatní pomocné funkce (beze změny) ---
function is_ok = check_runway_separation_rules_v3(candidate_flight, rwy_log, rwy_occ_after_land_s, arr_arr_sep_s, dep_arr_sep_s, ~)
    is_ok = true;
    
    last_op = [];
    if ~isempty(rwy_log)
        relevant_ops_indices = find([rwy_log.time] <= candidate_flight.actual_operation_time);
        if ~isempty(relevant_ops_indices)
            last_op = rwy_log(relevant_ops_indices(end));
        end
    end

    if isempty(last_op)
        return; 
    end

    if strcmp(candidate_flight.type, 'arrival') 
        cand_landing_time = candidate_flight.landing_time;
        if strcmp(last_op.type, 'arrival') 
            if cand_landing_time < (last_op.detail_time + arr_arr_sep_s)
                is_ok = false; return;
            end
            if cand_landing_time < (last_op.detail_time + rwy_occ_after_land_s) 
                 is_ok = false; return;
            end
        elseif strcmp(last_op.type, 'departure') 
            if cand_landing_time < last_op.detail_time 
                is_ok = false; return;
            end
            idx_A1_candidates = find(strcmp({rwy_log.type},'arrival') & [rwy_log.time] < last_op.time);
            if ~isempty(idx_A1_candidates)
                [~, closest_A1_idx_in_candidates] = max([rwy_log(idx_A1_candidates).time]);
                A1_landing_time = rwy_log(idx_A1_candidates(closest_A1_idx_in_candidates)).detail_time;
                if cand_landing_time < (A1_landing_time + 3*60) 
                    is_ok = false; return;
                end
            end
        end
    else 
        cand_takeoff_time = candidate_flight.takeoff_time;

        if strcmp(last_op.type, 'arrival') 
            if cand_takeoff_time < (last_op.detail_time + dep_arr_sep_s) || cand_takeoff_time < (last_op.detail_time + rwy_occ_after_land_s)
                is_ok = false; return;
            end
        elseif strcmp(last_op.type, 'departure') 
            if cand_takeoff_time < last_op.detail_time
                is_ok = false; return;
            end
        end
    end
end

function all_sep_ok = check_all_air_separations_v3(candidate_flight, existing_flights, arr_routes, dep_route, min_sep_nm, time_to_cv314)
    all_sep_ok = true;
    if isempty(existing_flights), return; end

    for i = 1:length(existing_flights)
        existing_flight = existing_flights(i);
        
        if strcmp(candidate_flight.type, 'arrival')
            cand_start_air = candidate_flight.entry_time;
            cand_end_air = candidate_flight.landing_time;
        else 
            cand_start_air = candidate_flight.takeoff_time;
            cand_end_air = candidate_flight.exit_time;
        end
        if strcmp(existing_flight.type, 'arrival')
            exist_start_air = existing_flight.entry_time;
            exist_end_air = existing_flight.landing_time;
        else 
            exist_start_air = existing_flight.takeoff_time;
            exist_end_air = existing_flight.exit_time;
        end
        
        overlap_start_time = max(cand_start_air, exist_start_air);
        overlap_end_time = min(cand_end_air, exist_end_air);

        if overlap_start_time >= overlap_end_time, continue; end 

        for t_check = overlap_start_time:15:overlap_end_time 
            pos_cand = get_flight_position_at_time_v3(candidate_flight, t_check, arr_routes, dep_route, time_to_cv314);
            pos_exist = get_flight_position_at_time_v3(existing_flight, t_check, arr_routes, dep_route, time_to_cv314);

            if isempty(pos_cand) || isempty(pos_exist), continue; end 

            dist_nm = haversine_distance(pos_cand.lat, pos_cand.lon, pos_exist.lat, pos_exist.lon);
            if dist_nm < min_sep_nm
                all_sep_ok = false;
                return;
            end
        end
    end
end

function position = get_flight_position_at_time_v3(flight, abs_time, arr_routes, dep_route, time_to_cv314)
    position = []; 
    if strcmp(flight.type, 'arrival')
        if abs_time >= flight.entry_time && abs_time <= flight.landing_time
            time_in_route = abs_time - flight.entry_time;
            route_details = arr_routes{flight.route_index};
            dist_traveled = calculate_distance_traveled_generic_v3(time_in_route, route_details);
            [lat, lon, speed] = interpolate_position_generic_v3(route_details, dist_traveled);
            position.lat = lat; position.lon = lon; position.speed = speed;
        end
    else 
        if abs_time >= flight.takeoff_time && abs_time <= flight.exit_time
            time_since_takeoff = abs_time - flight.takeoff_time;
            dist_traveled = calculate_distance_traveled_departure_v3(time_since_takeoff, dep_route, time_to_cv314);
            [lat, lon, speed] = interpolate_position_departure_v3(dep_route, dist_traveled, time_to_cv314);
            position.lat = lat; position.lon = lon; position.speed = speed;
        end
    end
end

function dist_traveled = calculate_distance_traveled_generic_v3(time_in_route, route_details)
    if time_in_route <= 0, dist_traveled = 0; return; end
    if isempty(route_details.segment_time), dist_traveled = 0; return; end
    
    accumulated_time = 0; accumulated_distance = 0;
    for i = 1:length(route_details.segment_time)
        segment_end_time = accumulated_time + route_details.segment_time(i);
        if time_in_route <= segment_end_time || i == length(route_details.segment_time)
            frac = 0;
            if route_details.segment_time(i) > 1e-6 
                 frac = min(1, (time_in_route - accumulated_time) / route_details.segment_time(i));
            end
            dist_traveled = accumulated_distance + frac * route_details.segment_dist(i);
            return;
        end
        accumulated_time = segment_end_time;
        accumulated_distance = accumulated_distance + route_details.segment_dist(i);
    end
    dist_traveled = accumulated_distance; 
end

function [lat, lon, speed] = interpolate_position_generic_v3(route_details, dist_traveled)
    segment_start_dist = 0; segment_index = 1;
    if dist_traveled >= route_details.total_dist, dist_traveled = route_details.total_dist - 1e-6; end
    if dist_traveled < 0, dist_traveled = 0; end

    for i = 1:length(route_details.segment_dist)
        segment_end_dist = segment_start_dist + route_details.segment_dist(i);
        if dist_traveled <= segment_end_dist || i == length(route_details.segment_dist)
            segment_index = i;
            break;
        end
        segment_start_dist = segment_end_dist;
    end
    
    segment_rel_pos = 0;
    if route_details.segment_dist(segment_index) > 1e-6 
        segment_rel_pos = (dist_traveled - segment_start_dist) / route_details.segment_dist(segment_index);
    end
    segment_rel_pos = min(1, max(0, segment_rel_pos));
    
    wp1 = route_details.waypoint_coords(segment_index, :);
    wp2 = route_details.waypoint_coords(segment_index + 1, :);
    lat = wp1(1) + segment_rel_pos * (wp2(1) - wp1(1));
    lon = wp1(2) + segment_rel_pos * (wp2(2) - wp1(2));
    
    speed1 = route_details.speed_at_waypoints(segment_index);
    speed2 = route_details.speed_at_waypoints(segment_index + 1);
    speed = speed1 + segment_rel_pos * (speed2 - speed1);
end

function dist_traveled = calculate_distance_traveled_departure_v3(time_since_takeoff, dep_route_details, time_to_reach_cv314)
    if time_since_takeoff <= 0, dist_traveled = 0; return; end

    dist_rwy_cv314 = dep_route_details.segment_dist(1);
    knots_to_nm_per_sec_local = 1/3600; 
    
    if time_since_takeoff <= time_to_reach_cv314
        avg_speed_seg1 = dep_route_details.speed_at_waypoints(2) / 2 * knots_to_nm_per_sec_local; 
        dist_traveled = avg_speed_seg1 * time_since_takeoff;
    else
        time_after_cv314 = time_since_takeoff - time_to_reach_cv314;
        speed_after_cv314 = dep_route_details.speed_at_waypoints(2) * knots_to_nm_per_sec_local; 
        dist_traveled = dist_rwy_cv314 + speed_after_cv314 * time_after_cv314;
    end
    dist_traveled = min(dist_traveled, dep_route_details.total_dist); 
end

function [lat, lon, speed] = interpolate_position_departure_v3(dep_route_details, dist_traveled, ~)
    dist_rwy_cv314 = dep_route_details.segment_dist(1); 
    
    if dist_traveled <= dist_rwy_cv314 
        segment_rel_pos = 0;
        if dist_rwy_cv314 > 1e-6, segment_rel_pos = dist_traveled / dist_rwy_cv314; end
        segment_rel_pos = min(1, max(0, segment_rel_pos));
        
        wp1 = dep_route_details.waypoint_coords(1, :); 
        wp2 = dep_route_details.waypoint_coords(2, :); 
        lat = wp1(1) + segment_rel_pos * (wp2(1) - wp1(1));
        lon = wp1(2) + segment_rel_pos * (wp2(2) - wp1(2));
        
        speed_at_rwy = dep_route_details.speed_at_waypoints(1); 
        speed_at_cv314 = dep_route_details.speed_at_waypoints(2); 
        speed = speed_at_rwy + segment_rel_pos * (speed_at_cv314 - speed_at_rwy);
    else 
        dist_on_second_segment = dist_traveled - dist_rwy_cv314;
        dist_cv314_eruso = dep_route_details.segment_dist(2);
        
        segment_rel_pos = 0;
        if dist_cv314_eruso > 1e-6, segment_rel_pos = dist_on_second_segment / dist_cv314_eruso; end
        segment_rel_pos = min(1, max(0, segment_rel_pos));

        wp1 = dep_route_details.waypoint_coords(2, :); 
        wp2 = dep_route_details.waypoint_coords(3, :); 
        lat = wp1(1) + segment_rel_pos * (wp2(1) - wp1(1));
        lon = wp1(2) + segment_rel_pos * (wp2(2) - wp1(2));
        
        speed = dep_route_details.speed_at_waypoints(2); 
    end
end

function decimal_degrees = custom_dms2degrees(dms_str)
    sign_val = 1;
    if contains(dms_str, 'S') || contains(dms_str, 'W'), sign_val = -1; end
    
    dms_str_cleaned = regexprep(dms_str, '[NSEW]', '');
    
    if length(dms_str_cleaned) < 5 
        if length(dms_str_cleaned) == 4 
             dms_str_cleaned = [dms_str_cleaned, '00']; 
        elseif length(dms_str_cleaned) == 2 
             dms_str_cleaned = [dms_str_cleaned, '0000']; 
        end
    end
    
    deg_len = 2; 
    if ~(contains(dms_str, 'N') || contains(dms_str, 'S')) 
        deg_len = 3; 
    end
    if length(dms_str_cleaned) < deg_len + 4
        degrees = str2double(dms_str_cleaned(1:deg_len));
        minutes = 0;
        seconds = 0;
         if length(dms_str_cleaned) >= deg_len + 2
            minutes = str2double(dms_str_cleaned(deg_len+1:deg_len+2));
         end
    else
        degrees = str2double(dms_str_cleaned(1:deg_len));
        minutes = str2double(dms_str_cleaned(deg_len+1:deg_len+2));
        seconds = str2double(dms_str_cleaned(deg_len+3:end));
    end

    if isnan(degrees) || isnan(minutes) || isnan(seconds)
        warning('Chyba při převodu DMS na DD: %s', dms_str);
        decimal_degrees = NaN; return;
    end
    decimal_degrees = sign_val * (degrees + minutes/60 + seconds/3600);
end

function d = haversine_distance(lat1, lon1, lat2, lon2)
    lat1_rad = lat1 * pi/180; lon1_rad = lon1 * pi/180;
    lat2_rad = lat2 * pi/180; lon2_rad = lon2 * pi/180;
    R_nm = 3440.065; 
    a = sin((lat2_rad-lat1_rad)/2)^2 + cos(lat1_rad)*cos(lat2_rad)*sin((lon2_rad-lon1_rad)/2)^2;
    c = 2 * atan2(sqrt(a), sqrt(1-a));
    d = R_nm * c;
end

function arrival_times = generate_uniform_arrivals(num_aircraft, max_time, buffer_time_val)
    if num_aircraft <= 0, arrival_times = []; return; end
    usable_time = max_time - 2 * buffer_time_val;
    if usable_time <=0, usable_time = max_time; buffer_time_val = 0; end
    if num_aircraft == 1
        arrival_times = buffer_time_val + usable_time / 2;
    else
        interval = usable_time / (num_aircraft -1);
        arrival_times = buffer_time_val + (0:num_aircraft-1) * interval;
    end
    if num_aircraft > 0 && exist('interval','var') && interval > 0
        random_offset = (rand(1, num_aircraft) - 0.5) * 0.1 * min(interval,60);
        arrival_times = arrival_times + random_offset;
    end
    arrival_times = sort(max(0, min(max_time, arrival_times)));
    arrival_times = unique(arrival_times);
end

function departure_times = generate_uniform_departures(num_departures, max_time, buffer_time_val)
    if num_departures <= 0, departure_times = []; return; end
    usable_time = max_time - 2 * buffer_time_val;
    if usable_time <=0, usable_time = max_time; buffer_time_val = 0; end
    if num_departures == 1
        departure_times = buffer_time_val + usable_time / 2;
    else
        interval = usable_time / (num_departures-1);
        departure_times = buffer_time_val + (0:num_departures-1) * interval;
    end
     if num_departures > 0 && exist('interval','var') && interval > 0
        random_offset = (rand(1, num_departures) - 0.5) * 0.1 * min(interval,60);
        departure_times = departure_times + random_offset;
    end
    departure_times = sort(max(0, min(max_time, departure_times)));
    departure_times = unique(departure_times);
end

function route_idx_val = select_route(probabilities_val)
    r_val = rand(); cum_prob_val = 0; route_idx_val = length(probabilities_val);
    for i_val = 1:length(probabilities_val)
        cum_prob_val = cum_prob_val + probabilities_val(i_val);
        if r_val <= cum_prob_val, route_idx_val = i_val; return; end
    end
end