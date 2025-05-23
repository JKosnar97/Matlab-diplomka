% MATLAB Skript pro simulaci kapacity příletů a odletů v TMA Kbely
% Verze 4.0: Se střídavým plánováním (interleaving) z Čáslavi
% Implementace střídavého plánování příletů a odletů podle původního času

clear;
clc;
close all;

% ----------------- KONFIGURAČNÍ PARAMETRY -------------------
% Základní nastavení simulace
SIM_DURATION_S = 3600;            % Doba simulace v sekundách (1 hodina)
TIME_STEP_S = 1;                  % Časový krok simulace v sekundách
RANDOM_SEED = 42;                 % Pro reprodukovatelnost výsledků
DRAW_ANIMATION = true;            % Vykreslení animace pohybu letounů
ANIMATE_SPEED = 5;                % Rychlost animace (kolikrát rychleji než reálný čas)

% Parametry generování letounů
FIXED_NUM_AIRCRAFT = 40;          % Pevný počet letounů v simulaci pro přílety
FIXED_NUM_DEPARTURES = 40;        % Pevný počet letounů pro odlety
ROUTE_PROBABILITY = [0.5, 0.5];   % Pravděpodobnost výběru trasy (Trasa 1, Trasa 2)

% Parametry separace
MIN_SEPARATION_NM_AIR = 5;        % Minimální horizontální rozestup ve vzduchu v NM

% Pokročilá pravidla separace na dráze (převzato z Čáslavi)
RUNWAY_OCCUPANCY_AFTER_LANDING_S = 60;      % Doba obsazení dráhy po přistání
ARR_AFTER_ARR_MIN_INTERVAL_S = 120;         % Min. interval mezi přistáními
DEP_AFTER_ARR_MIN_INTERVAL_S = 60;          % Min. interval vzlet po přistání
ARR_AFTER_DEP_AFTER_ARR_MIN_INTERVAL_S = 180; % Speciální pravidlo A1-D1-A2

% Pracovní zátěž řídícího - 300bodová škála
WORKLOAD_INTERVAL_S = 60;         % Interval pro výpočet zátěže (1 minuta)
THEORETICAL_MAX_SCALE = 300;      % Teoretické maximum škály
OVERLOAD_70_PERCENT = 210;        % 70% z 300 = práh přetížení
MAX_WORKLOAD_POINTS = 210;        % Plánovací limit = práh přetížení
BASE_WORKLOAD_POINTS = 10;        % Základní zátěž
LOW_WORKLOAD_THRESHOLD = 60;      % Nízká zátěž
MEDIUM_WORKLOAD_THRESHOLD = 90;   % Střední zátěž
HIGH_WORKLOAD_THRESHOLD = 162;    % Vysoká zátěž
OVERLOAD_THRESHOLD = 210;         % Přetížení (70% z 300)

% Teoretická hodinová kapacita pro statistiky
THEORETICAL_HOURLY_CAPACITY = 60; % Upravte podle potřeby

rng(RANDOM_SEED);

% ----------------- DEFINICE KOMUNIKAČNÍ ZÁTĚŽE (PŘEVZATO Z ČÁSLAVI) -------------------
comm_load_params = struct();

% PŘÍLETOVÉ KOMUNIKACE
% Předběžný kontakt (původně pre_approach_contact v Čáslavi)
comm_load_params.initial_contact.duration_total_s = 90;
comm_load_params.initial_contact.controller_workload_points = 31; % 35×0.9
comm_load_params.initial_contact.timing_offset_s = -3 * 60; % -3 minuty PŘED vstupem

% Předání na věž při FAF (původně handoff_to_tower v Čáslavi)
comm_load_params.faf_report.duration_total_s = 10;
comm_load_params.faf_report.controller_workload_points = 13; % 15×0.9
comm_load_params.faf_report.timing_offset_s = 0; % při průletu FAF

% ODLETOVÉ KOMUNIKACE
% Koordinace věž-radar (původně tower_radar_coordination v Čáslavi)
comm_load_params.takeoff_clearance.duration_total_s = 20;
comm_load_params.takeoff_clearance.controller_workload_points = 22; % 25×0.9
comm_load_params.takeoff_clearance.timing_offset_s = -1 * 60; % -1 minuta před vzletem

% Kontakt po vzletu při VECTOR1 (původně post_takeoff_contact v Čáslavi)
comm_load_params.departure_contact.duration_total_s = 15;
comm_load_params.departure_contact.controller_workload_points = 22; % 25×0.9
comm_load_params.departure_contact.timing_offset_s = 0; % při průletu VECTOR1

% Finální kontakt při EXIT (původně final_contact v Čáslavi)
comm_load_params.exit_clearance.duration_total_s = 15;
comm_load_params.exit_clearance.controller_workload_points = 13; % 15×0.9
comm_load_params.exit_clearance.timing_offset_s = 0; % při průletu EXIT

% ----------------- 1. DEFINICE VSTUPNÍCH PARAMETRŮ -------------------
disp('--- Definice vstupních parametrů ---');
dms_to_dd = @(dms_str) custom_dms2degrees(dms_str);
coords = struct();
earth_radius_nm = 3440.065;

% Definice klíčových bodů (zachováno z Kbely)
coords.EKROT = [dms_to_dd('500346.00N'), dms_to_dd('0145313.00E')];
coords.RWY24_THR = [dms_to_dd('500731.93N'), dms_to_dd('0143321.70E')];
coords.RWY_ARR = coords.RWY24_THR;
coords.RWY_DEP = coords.RWY24_THR;

% Výpočet FAF
lat_rwy24_rad_faf_calc = coords.RWY24_THR(1) * pi/180;
lon_rwy24_rad_faf_calc = coords.RWY24_THR(2) * pi/180;
bearing_to_faf_from_rwy_rad = (237 + 180) * pi/180;
dist_nm_faf_from_rwy = 5.8;
faf_lat_rad = asin(sin(lat_rwy24_rad_faf_calc) * cos(dist_nm_faf_from_rwy/earth_radius_nm) + cos(lat_rwy24_rad_faf_calc) * sin(dist_nm_faf_from_rwy/earth_radius_nm) * cos(bearing_to_faf_from_rwy_rad));
faf_lon_rad = lon_rwy24_rad_faf_calc + atan2(sin(bearing_to_faf_from_rwy_rad) * sin(dist_nm_faf_from_rwy/earth_radius_nm) * cos(lat_rwy24_rad_faf_calc), cos(dist_nm_faf_from_rwy/earth_radius_nm) - sin(lat_rwy24_rad_faf_calc) * sin(faf_lat_rad));
coords.FAF = [faf_lat_rad * 180/pi, faf_lon_rad * 180/pi];

% Výpočet vstupních bodů a ostatních bodů (zachováno z Kbely)
coords.VLM = [dms_to_dd('494215.38N'), dms_to_dd('0150400.27E')];
bearing_PR522_rad = 303 * pi/180;
dist_nm_PR522_from_VLM = 11.0;
lat_vlm_rad = coords.VLM(1) * pi/180;
lon_vlm_rad = coords.VLM(2) * pi/180;
pr522_lat_rad = asin(sin(lat_vlm_rad) * cos(dist_nm_PR522_from_VLM/earth_radius_nm) + cos(lat_vlm_rad) * sin(dist_nm_PR522_from_VLM/earth_radius_nm) * cos(bearing_PR522_rad));
pr522_lon_rad = lon_vlm_rad + atan2(sin(bearing_PR522_rad) * sin(dist_nm_PR522_from_VLM/earth_radius_nm) * cos(lat_vlm_rad), cos(dist_nm_PR522_from_VLM/earth_radius_nm) - sin(lat_vlm_rad) * sin(pr522_lat_rad));
coords.PR522 = [pr522_lat_rad * 180/pi, pr522_lon_rad * 180/pi];

% Výpočet ENTRY_PR522
course_PR522_EKROT_rad = calculate_initial_course(coords.PR522(1), coords.PR522(2), coords.EKROT(1), coords.EKROT(2)) * pi/180;
dist_nm_from_pr522_to_entry = 3.0;
entry_pr522_lat_rad = asin(sin(pr522_lat_rad) * cos(dist_nm_from_pr522_to_entry/earth_radius_nm) + cos(pr522_lat_rad) * sin(dist_nm_from_pr522_to_entry/earth_radius_nm) * cos(course_PR522_EKROT_rad));
entry_pr522_lon_rad = pr522_lon_rad + atan2(sin(course_PR522_EKROT_rad) * sin(dist_nm_from_pr522_to_entry/earth_radius_nm) * cos(pr522_lat_rad), cos(dist_nm_from_pr522_to_entry/earth_radius_nm) - sin(pr522_lat_rad) * sin(entry_pr522_lat_rad));
coords.ENTRY_PR522 = [entry_pr522_lat_rad * 180/pi, entry_pr522_lon_rad * 180/pi];

% Výpočet ELPON a ENTRY_ELPON
coords.ELPON = [dms_to_dd('495530.04N'), dms_to_dd('0143702.29E')];
bearing_to_ekrot_from_elpon_rad = 47 * pi/180;
dist_nm_from_elpon_to_entry = 3.0;
lat_elpon_rad = coords.ELPON(1) * pi/180;
lon_elpon_rad = coords.ELPON(2) * pi/180;
entry_elpon_lat_rad = asin(sin(lat_elpon_rad) * cos(dist_nm_from_elpon_to_entry/earth_radius_nm) + cos(lat_elpon_rad) * sin(dist_nm_from_elpon_to_entry/earth_radius_nm) * cos(bearing_to_ekrot_from_elpon_rad));
entry_elpon_lon_rad = lon_elpon_rad + atan2(sin(bearing_to_ekrot_from_elpon_rad) * sin(dist_nm_from_elpon_to_entry/earth_radius_nm) * cos(lat_elpon_rad), cos(dist_nm_from_elpon_to_entry/earth_radius_nm) - sin(lat_elpon_rad) * sin(entry_elpon_lat_rad));
coords.ENTRY_ELPON = [entry_elpon_lat_rad * 180/pi, entry_elpon_lon_rad * 180/pi];

% Odletové body
bearing_vector1_rad = 240 * pi/180;
dist_nm_vector1 = 2.0;
lat_rwy24_rad_vec_calc = coords.RWY24_THR(1) * pi/180;
lon_rwy24_rad_vec_calc = coords.RWY24_THR(2) * pi/180;
vector1_lat_rad = asin(sin(lat_rwy24_rad_vec_calc) * cos(dist_nm_vector1/earth_radius_nm) + cos(lat_rwy24_rad_vec_calc) * sin(dist_nm_vector1/earth_radius_nm) * cos(bearing_vector1_rad));
vector1_lon_rad = lon_rwy24_rad_vec_calc + atan2(sin(bearing_vector1_rad) * sin(dist_nm_vector1/earth_radius_nm) * cos(lat_rwy24_rad_vec_calc), cos(dist_nm_vector1/earth_radius_nm) - sin(lat_rwy24_rad_vec_calc) * sin(vector1_lat_rad));
coords.VECTOR1 = [vector1_lat_rad * 180/pi, vector1_lon_rad * 180/pi];

course_VECTOR1_EKROT_rad = calculate_initial_course(coords.VECTOR1(1), coords.VECTOR1(2), coords.EKROT(1), coords.EKROT(2)) * pi/180;
dist_nm_EKROT_EXIT = 6.0;
lat_ekrot_rad = coords.EKROT(1) * pi/180;
lon_ekrot_rad = coords.EKROT(2) * pi/180;
exit_lat_rad = asin(sin(lat_ekrot_rad) * cos(dist_nm_EKROT_EXIT/earth_radius_nm) + cos(lat_ekrot_rad) * sin(dist_nm_EKROT_EXIT/earth_radius_nm) * cos(course_VECTOR1_EKROT_rad));
exit_lon_rad = lon_ekrot_rad + atan2(sin(course_VECTOR1_EKROT_rad) * sin(dist_nm_EKROT_EXIT/earth_radius_nm) * cos(lat_ekrot_rad), cos(dist_nm_EKROT_EXIT/earth_radius_nm) - sin(lat_ekrot_rad) * sin(exit_lat_rad));
coords.EXIT = [exit_lat_rad * 180/pi, exit_lon_rad * 180/pi];

% Výpočet vzdáleností (zachováno z Kbely)
dist_exact = struct();
dist_exact.ENTRY_PR522_EKROT = 11.9;
dist_exact.ENTRY_ELPON_EKROT = 10.3;
dist_exact.EKROT_FAF = haversine_distance(coords.EKROT(1), coords.EKROT(2), coords.FAF(1), coords.FAF(2));
dist_exact.FAF_RWY_ARR = haversine_distance(coords.FAF(1), coords.FAF(2), coords.RWY_ARR(1), coords.RWY_ARR(2));
dist_exact.RWY_DEP_VECTOR1 = 2.0;
dist_exact.VECTOR1_EKROT = haversine_distance(coords.VECTOR1(1), coords.VECTOR1(2), coords.EKROT(1), coords.EKROT(2));
dist_exact.EKROT_EXIT = haversine_distance(coords.EKROT(1), coords.EKROT(2), coords.EXIT(1), coords.EXIT(2));

% Rychlostní profily a výpočet časů (UPRAVENÉ RYCHLOSTI)
knots_to_nm_per_sec = 1/3600;
speed_profile_1 = struct('ENTRY_PR522', 250, 'EKROT', 200, 'FAF', 160, 'RWY_ARR', 130);
speed_profile_2 = struct('ENTRY_ELPON', 250, 'EKROT', 200, 'FAF', 160, 'RWY_ARR', 130);
speed_profile_dep = struct('RWY_DEP', 0, 'VECTOR1', 150, 'EKROT', 210, 'EXIT', 250);

% Výpočet časů průletu
t1_ENTRY_PR522_EKROT_s = dist_exact.ENTRY_PR522_EKROT / (mean([speed_profile_1.ENTRY_PR522, speed_profile_1.EKROT]) * knots_to_nm_per_sec);
t1_EKROT_FAF_s = dist_exact.EKROT_FAF / (mean([speed_profile_1.EKROT, speed_profile_1.FAF]) * knots_to_nm_per_sec);
t1_FAF_RWY_s = dist_exact.FAF_RWY_ARR / (mean([speed_profile_1.FAF, speed_profile_1.RWY_ARR]) * knots_to_nm_per_sec);
t1_total_s = t1_ENTRY_PR522_EKROT_s + t1_EKROT_FAF_s + t1_FAF_RWY_s;

t2_ENTRY_ELPON_EKROT_s = dist_exact.ENTRY_ELPON_EKROT / (mean([speed_profile_2.ENTRY_ELPON, speed_profile_2.EKROT]) * knots_to_nm_per_sec);
t2_EKROT_FAF_s = dist_exact.EKROT_FAF / (mean([speed_profile_2.EKROT, speed_profile_2.FAF]) * knots_to_nm_per_sec);
t2_FAF_RWY_s = dist_exact.FAF_RWY_ARR / (mean([speed_profile_2.FAF, speed_profile_2.RWY_ARR]) * knots_to_nm_per_sec);
t2_total_s = t2_ENTRY_ELPON_EKROT_s + t2_EKROT_FAF_s + t2_FAF_RWY_s;

td_RWY_VEC_s = dist_exact.RWY_DEP_VECTOR1 / (mean([speed_profile_dep.RWY_DEP, speed_profile_dep.VECTOR1]) * knots_to_nm_per_sec);
td_VEC_EKR_s = dist_exact.VECTOR1_EKROT / (mean([speed_profile_dep.VECTOR1, speed_profile_dep.EKROT]) * knots_to_nm_per_sec);
td_EKR_EXIT_s = dist_exact.EKROT_EXIT / (mean([speed_profile_dep.EKROT, speed_profile_dep.EXIT]) * knots_to_nm_per_sec);
td_total_s = td_RWY_VEC_s + td_VEC_EKR_s + td_EKR_EXIT_s;

% Definice struktur tratí
route1 = struct('name','ENTRY_PR522-EKROT-FAF-THR RWY24','waypoints',{{'ENTRY_PR522','EKROT','FAF','RWY_ARR'}},'waypoint_coords',[coords.ENTRY_PR522;coords.EKROT;coords.FAF;coords.RWY_ARR],'segment_dist',[dist_exact.ENTRY_PR522_EKROT,dist_exact.EKROT_FAF,dist_exact.FAF_RWY_ARR],'segment_time',[t1_ENTRY_PR522_EKROT_s,t1_EKROT_FAF_s,t1_FAF_RWY_s],'total_dist',sum([dist_exact.ENTRY_PR522_EKROT,dist_exact.EKROT_FAF,dist_exact.FAF_RWY_ARR]),'total_time',t1_total_s,'speed_at_waypoints',[speed_profile_1.ENTRY_PR522,speed_profile_1.EKROT,speed_profile_1.FAF,speed_profile_1.RWY_ARR]);
route2 = struct('name','ENTRY_ELPON-EKROT-FAF-THR RWY24','waypoints',{{'ENTRY_ELPON','EKROT','FAF','RWY_ARR'}},'waypoint_coords',[coords.ENTRY_ELPON;coords.EKROT;coords.FAF;coords.RWY_ARR],'segment_dist',[dist_exact.ENTRY_ELPON_EKROT,dist_exact.EKROT_FAF,dist_exact.FAF_RWY_ARR],'segment_time',[t2_ENTRY_ELPON_EKROT_s,t2_EKROT_FAF_s,t2_FAF_RWY_s],'total_dist',sum([dist_exact.ENTRY_ELPON_EKROT,dist_exact.EKROT_FAF,dist_exact.FAF_RWY_ARR]),'total_time',t2_total_s,'speed_at_waypoints',[speed_profile_2.ENTRY_ELPON,speed_profile_2.EKROT,speed_profile_2.FAF,speed_profile_2.RWY_ARR]);
route_dep = struct('name','THR RWY24-VECTOR1-EKROT-EXIT','waypoints',{{'RWY_DEP','VECTOR1','EKROT','EXIT'}},'waypoint_coords',[coords.RWY_DEP;coords.VECTOR1;coords.EKROT;coords.EXIT],'segment_dist',[dist_exact.RWY_DEP_VECTOR1,dist_exact.VECTOR1_EKROT,dist_exact.EKROT_EXIT],'segment_time',[td_RWY_VEC_s,td_VEC_EKR_s,td_EKR_EXIT_s],'total_dist',sum([dist_exact.RWY_DEP_VECTOR1,dist_exact.VECTOR1_EKROT,dist_exact.EKROT_EXIT]),'total_time',td_total_s,'speed_at_waypoints',[speed_profile_dep.RWY_DEP,speed_profile_dep.VECTOR1,speed_profile_dep.EKROT,speed_profile_dep.EXIT]);
arrival_routes = {route1, route2};

disp(['Rychlosti: ENTRY-EKROT (', num2str(speed_profile_1.ENTRY_PR522), '-', num2str(speed_profile_1.EKROT), '), EKROT-FAF (', num2str(speed_profile_1.EKROT), '-', num2str(speed_profile_1.FAF), '), FAF-THR (', num2str(speed_profile_1.FAF), '-', num2str(speed_profile_1.RWY_ARR), ')']);

% ----------------- 2. GENEROVÁNÍ A PLÁNOVÁNÍ LETŮ -------------------
disp(' ');
disp('--- Generování a plánování letů (Střídavé plánování) ---');
disp(['Lety: ', num2str(FIXED_NUM_AIRCRAFT), ' příletů + ', num2str(FIXED_NUM_DEPARTURES), ' odletů']);
disp(['Separace: ', num2str(MIN_SEPARATION_NM_AIR), ' NM, Limit zátěže: ', num2str(MAX_WORKLOAD_POINTS), ' bodů']);

buffer_time = 60;
orig_arrival_times = generate_uniform_arrivals(FIXED_NUM_AIRCRAFT, SIM_DURATION_S, buffer_time);
orig_departure_times = generate_uniform_departures(FIXED_NUM_DEPARTURES, SIM_DURATION_S, buffer_time);

% STŘÍDAVÉ PLÁNOVÁNÍ - implementace z Čáslavi
[scheduled_flights, rejected_flights_info, final_workload_per_interval, runway_operations_log] = ...
    plan_flights_with_interleaved_workload_control_kbely(orig_arrival_times, orig_departure_times, ...
    arrival_routes, route_dep, ROUTE_PROBABILITY, SIM_DURATION_S, WORKLOAD_INTERVAL_S, ...
    MAX_WORKLOAD_POINTS, BASE_WORKLOAD_POINTS, comm_load_params, MIN_SEPARATION_NM_AIR, ...
    RUNWAY_OCCUPANCY_AFTER_LANDING_S, ARR_AFTER_ARR_MIN_INTERVAL_S, DEP_AFTER_ARR_MIN_INTERVAL_S, td_RWY_VEC_s);

% Rozdělení na přílety a odlety
scheduled_arrivals = [];
scheduled_departures = [];
if ~isempty(scheduled_flights)
    scheduled_arrivals = scheduled_flights(strcmp({scheduled_flights.type}, 'arrival'));
    scheduled_departures = scheduled_flights(strcmp({scheduled_flights.type}, 'departure'));
end

disp(['Naplánováno: ', num2str(length(scheduled_arrivals)), ' příletů a ', ...
      num2str(length(scheduled_departures)), ' odletů (střídavé plánování)']);
if ~isempty(rejected_flights_info)
    disp(['Odmítnuto: ', num2str(length(rejected_flights_info)), ' letů (', num2str(length(rejected_flights_info)/(FIXED_NUM_AIRCRAFT+FIXED_NUM_DEPARTURES)*100, '%.1f'), '%)']);
end

% Příprava komunikačních událostí pro dynamickou simulaci
aircraft_comm_events_for_dyn_sim = struct('time', {}, 'points', {}, 'flight_id', {}, 'duration', {});
if ~isempty(scheduled_flights)
    for i = 1:length(scheduled_flights)
        flight = scheduled_flights(i);
        if strcmp(flight.type, 'arrival')
            % PŘÍLETOVÉ KOMUNIKACE (pouze 2 události jako v Čáslavi)
            aircraft_comm_events_for_dyn_sim(end+1) = struct('time', flight.entry_time + comm_load_params.initial_contact.timing_offset_s, 'points', comm_load_params.initial_contact.controller_workload_points, 'flight_id', flight.id, 'duration', comm_load_params.initial_contact.duration_total_s);
            aircraft_comm_events_for_dyn_sim(end+1) = struct('time', flight.faf_time + comm_load_params.faf_report.timing_offset_s, 'points', comm_load_params.faf_report.controller_workload_points, 'flight_id', flight.id, 'duration', comm_load_params.faf_report.duration_total_s);
        else
            % ODLETOVÉ KOMUNIKACE (3 události jako v Čáslavi)
            aircraft_comm_events_for_dyn_sim(end+1) = struct('time', flight.takeoff_time + comm_load_params.takeoff_clearance.timing_offset_s, 'points', comm_load_params.takeoff_clearance.controller_workload_points, 'flight_id', flight.id, 'duration', comm_load_params.takeoff_clearance.duration_total_s);
            aircraft_comm_events_for_dyn_sim(end+1) = struct('time', flight.vector1_time + comm_load_params.departure_contact.timing_offset_s, 'points', comm_load_params.departure_contact.controller_workload_points, 'flight_id', flight.id, 'duration', comm_load_params.departure_contact.duration_total_s);
            aircraft_comm_events_for_dyn_sim(end+1) = struct('time', flight.exit_time + comm_load_params.exit_clearance.timing_offset_s, 'points', comm_load_params.exit_clearance.controller_workload_points, 'flight_id', flight.id, 'duration', comm_load_params.exit_clearance.duration_total_s);
        end
    end
    if ~isempty(aircraft_comm_events_for_dyn_sim)
        [~, sort_idx_comm] = sort([aircraft_comm_events_for_dyn_sim.time]);
        aircraft_comm_events_for_dyn_sim = aircraft_comm_events_for_dyn_sim(sort_idx_comm);
    end
end

% ----------------- 3. INICIALIZACE STATISTICKÝCH PROMĚNNÝCH -------------------
active_aircraft_count_per_interval = zeros(1, ceil(SIM_DURATION_S / WORKLOAD_INTERVAL_S));

% ----------------- 4. DYNAMICKÁ SIMULACE A VIZUALIZACE -------------------
disp(' ');
disp('--- Dynamická simulace ---');
landing_count = 0; takeoff_count = 0;

if DRAW_ANIMATION
    figure('Name','TMA Kbely - Střídavé plánování','Position',[100,100,1200,800]); hold on;
    
    % Vykreslení bodů (zachováno z původního Kbely)
    plot(coords.VLM(2),coords.VLM(1),'ko','MarkerSize',10,'MarkerFaceColor','k'); text(coords.VLM(2)+0.003,coords.VLM(1)+0.003,'VLM','FontSize',10);
    plot(coords.PR522(2),coords.PR522(1),'bo','MarkerSize',10,'MarkerFaceColor',[0.8,0.8,1]); text(coords.PR522(2)+0.003,coords.PR522(1)+0.003,'PR522 (Ref)','FontSize',10);
    plot(coords.ENTRY_PR522(2),coords.ENTRY_PR522(1),'bo','MarkerSize',8,'MarkerFaceColor',[0.3,0.3,1]); text(coords.ENTRY_PR522(2)+0.003,coords.ENTRY_PR522(1)+0.003,'ENTRY_PR522','FontSize',8);
    plot(coords.ELPON(2),coords.ELPON(1),'go','MarkerSize',10,'MarkerFaceColor',[0.8,1,0.8]); text(coords.ELPON(2)+0.003,coords.ELPON(1)+0.003,'ELPON (Ref)','FontSize',10);
    plot(coords.ENTRY_ELPON(2),coords.ENTRY_ELPON(1),'go','MarkerSize',8,'MarkerFaceColor',[0.3,1,0.3]); text(coords.ENTRY_ELPON(2)+0.003,coords.ENTRY_ELPON(1)+0.003,'ENTRY_ELPON','FontSize',8);
    plot(coords.EKROT(2),coords.EKROT(1),'ko','MarkerSize',10,'MarkerFaceColor',[0.6,0.6,0.6]); text(coords.EKROT(2)+0.003,coords.EKROT(1)+0.003,'EKROT','FontSize',10);
    plot(coords.FAF(2),coords.FAF(1),'ro','MarkerSize',12,'MarkerFaceColor','r'); text(coords.FAF(2)+0.003,coords.FAF(1)+0.003,'FAF','FontSize',10);
    plot(coords.RWY_ARR(2),coords.RWY_ARR(1),'ks','MarkerSize',12,'MarkerFaceColor','k'); text(coords.RWY_ARR(2)+0.003,coords.RWY_ARR(1)-0.003,'THR RWY24','FontSize',10);
    plot(coords.VECTOR1(2),coords.VECTOR1(1),'mo','MarkerSize',10,'MarkerFaceColor','m'); text(coords.VECTOR1(2)+0.003,coords.VECTOR1(1)+0.003,'VECTOR1','FontSize',10);
    plot(coords.EXIT(2),coords.EXIT(1),'co','MarkerSize',10,'MarkerFaceColor','c'); text(coords.EXIT(2)+0.003,coords.EXIT(1)+0.003,'EXIT','FontSize',10);
    
    % Vykreslení tratí
    plot([coords.ENTRY_PR522(2),coords.EKROT(2),coords.FAF(2),coords.RWY_ARR(2)],[coords.ENTRY_PR522(1),coords.EKROT(1),coords.FAF(1),coords.RWY_ARR(1)],'--b','LineWidth',1.5);
    plot([coords.ENTRY_ELPON(2),coords.EKROT(2),coords.FAF(2),coords.RWY_ARR(2)],[coords.ENTRY_ELPON(1),coords.EKROT(1),coords.FAF(1),coords.RWY_ARR(1)],'--g','LineWidth',1.5);
    plot([coords.RWY_DEP(2),coords.VECTOR1(2),coords.EKROT(2),coords.EXIT(2)],[coords.RWY_DEP(1),coords.VECTOR1(1),coords.EKROT(1),coords.EXIT(1)],'--m','LineWidth',1.5);

    xlabel('Zeměpisná délka (°E)'); ylabel('Zeměpisná šířka (°N)'); 
    title(['TMA Kbely - Střídavé plánování - Separace ',num2str(MIN_SEPARATION_NM_AIR),' NM']); 
    grid on; axis equal;
    
    % Nastavení rozsahu os
    all_lats_viz=[coords.PR522(1),coords.ENTRY_PR522(1),coords.ELPON(1),coords.ENTRY_ELPON(1),coords.EKROT(1),coords.FAF(1),coords.RWY_ARR(1),coords.VECTOR1(1),coords.EXIT(1),coords.VLM(1)]; 
    all_lons_viz=[coords.PR522(2),coords.ENTRY_PR522(2),coords.ELPON(2),coords.ENTRY_ELPON(2),coords.EKROT(2),coords.FAF(2),coords.RWY_ARR(2),coords.VECTOR1(2),coords.EXIT(2),coords.VLM(2)];
    lat_range_viz=max(all_lats_viz)-min(all_lats_viz); lon_range_viz=max(all_lons_viz)-min(all_lons_viz);
    lat_buffer_viz=lat_range_viz*0.15; lon_buffer_viz=lon_range_viz*0.15;
    if lat_buffer_viz==0,lat_buffer_viz=0.02;end; if lon_buffer_viz==0,lon_buffer_viz=0.02;end
    xlim([min(all_lons_viz)-lon_buffer_viz,max(all_lons_viz)+lon_buffer_viz]);
    ylim([min(all_lats_viz)-lat_buffer_viz,max(all_lats_viz)+lat_buffer_viz]);
    
    % Inicializace grafických objektů pro letadla
    arrival_markers=gobjects(length(scheduled_arrivals),1);
    arrival_labels=gobjects(length(scheduled_arrivals),1);
    if ~isempty(scheduled_arrivals)
        for i=1:length(scheduled_arrivals)
            route_idx_viz=scheduled_arrivals(i).route_index;
            marker_color_viz='b'; if route_idx_viz==2, marker_color_viz=[0 0.6 0]; end
            arrival_markers(i)=plot(NaN,NaN,'o','MarkerSize',8,'MarkerFaceColor',marker_color_viz,'MarkerEdgeColor',marker_color_viz);
            arrival_labels(i)=text(NaN,NaN,'','FontSize',8,'Color',marker_color_viz);
        end
    end
    departure_markers=gobjects(length(scheduled_departures),1);
    departure_labels=gobjects(length(scheduled_departures),1);
    if ~isempty(scheduled_departures)
        for i=1:length(scheduled_departures)
            departure_markers(i)=plot(NaN,NaN,'mo','MarkerSize',8,'MarkerFaceColor','m');
            departure_labels(i)=text(NaN,NaN,'','FontSize',8,'Color','m');
        end
    end
    
    % Indikátory
    separation_indicator=annotation('textbox',[0.02,0.92,0.3,0.06],'String','','FontSize',12,'EdgeColor','none','BackgroundColor','g');
    time_indicator=annotation('textbox',[0.7,0.92,0.28,0.06],'String','Čas: 0:00:00','FontSize',12,'EdgeColor','none');
    workload_indicator=annotation('textbox',[0.35,0.92,0.3,0.06],'String','Zátěž řídícího: 0','FontSize',12,'EdgeColor','none','BackgroundColor',[0.8,0.8,0.8]);
    
    % Inicializace pozic
    arrival_positions=repmat(struct('active',false,'lat',0,'lon',0,'speed',0,'id','','landed',false),length(scheduled_arrivals),1);
    departure_positions=repmat(struct('active',false,'lat',0,'lon',0,'speed',0,'id','','taken_off',false,'exited',false),length(scheduled_departures),1);
    
    disp('Spouštím animaci (5x rychlost)...');
    time_step_viz=TIME_STEP_S*ANIMATE_SPEED;

    current_dynamic_workload_profile = ones(1, ceil(SIM_DURATION_S / WORKLOAD_INTERVAL_S)) * BASE_WORKLOAD_POINTS;

    % Hlavní animační smyčka
    for current_time=0:time_step_viz:SIM_DURATION_S
        hours=floor(current_time/3600); minutes=floor(mod(current_time/60,60)); seconds=floor(mod(current_time,60));
        time_indicator.String=['Čas: ',sprintf('%d:%02d:%02d',hours,minutes,seconds)];
        
        % Zjištění aktivních letadel
        active_aircraft_positions = [];
        active_flights_now = struct('id',{},'lat',{},'lon',{});
        conflicts_detected = false;
        
        % Aktualizace příletů
        if ~isempty(scheduled_arrivals) && ~isempty(arrival_positions)
            for i=1:length(scheduled_arrivals)
                time_since_entry=current_time-scheduled_arrivals(i).entry_time;
                time_to_landing=scheduled_arrivals(i).landing_time-current_time;
                
                if time_since_entry>=0 && time_to_landing>=-time_step_viz
                    arrival_positions(i).active=true;
                    if isempty(arrival_positions(i).id), arrival_positions(i).id = scheduled_arrivals(i).id; end
                    dist_traveled=calculate_distance_traveled_kbely(scheduled_arrivals(i).route_index,time_since_entry,arrival_routes);
                    [lat,lon,speed]=interpolate_position_kbely(arrival_routes{scheduled_arrivals(i).route_index},dist_traveled);
                    arrival_positions(i).lat=lat; arrival_positions(i).lon=lon; arrival_positions(i).speed=speed;
                    set(arrival_markers(i),'XData',lon,'YData',lat,'Visible','on');
                    set(arrival_labels(i),'Position',[lon+0.002,lat+0.002],'String',sprintf('%s\n%dkt',scheduled_arrivals(i).id,round(speed)),'Visible','on');
                    active_aircraft_positions(end+1,:) = [lat, lon];
                    active_flights_now(end+1) = struct('id',scheduled_arrivals(i).id,'lat',lat,'lon',lon);
                    
                    if time_to_landing<=time_step_viz && time_to_landing>-time_step_viz && ~arrival_positions(i).landed
                        landing_count=landing_count+1; arrival_positions(i).landed=true;
                    end
                    if time_to_landing<0, arrival_positions(i).active=false; set(arrival_markers(i),'Visible','off'); set(arrival_labels(i),'Visible','off'); end
                else
                    arrival_positions(i).active=false; set(arrival_markers(i),'Visible','off'); set(arrival_labels(i),'Visible','off');
                end
            end
        end
        
        % Aktualizace odletů
        if ~isempty(scheduled_departures) && ~isempty(departure_positions)
            for i=1:length(scheduled_departures)
                time_since_takeoff=current_time-scheduled_departures(i).takeoff_time;
                time_to_exit=scheduled_departures(i).exit_time-current_time;
                
                if time_since_takeoff>=0 && time_to_exit>=-time_step_viz
                    departure_positions(i).active=true;
                    if isempty(departure_positions(i).id), departure_positions(i).id = scheduled_departures(i).id; end
                    dist_traveled_dep=calculate_distance_traveled_departure_kbely(time_since_takeoff,route_dep);
                    [lat,lon,speed]=interpolate_position_departure_kbely(route_dep,dist_traveled_dep);
                    departure_positions(i).lat=lat; departure_positions(i).lon=lon; departure_positions(i).speed=speed;
                    set(departure_markers(i),'XData',lon,'YData',lat,'Visible','on');
                    set(departure_labels(i),'Position',[lon+0.002,lat+0.002],'String',sprintf('%s\n%dkt',scheduled_departures(i).id,round(speed)),'Visible','on');
                    active_aircraft_positions(end+1,:) = [lat, lon];
                    active_flights_now(end+1) = struct('id',scheduled_departures(i).id,'lat',lat,'lon',lon);
                    
                    if time_since_takeoff<=time_step_viz && time_since_takeoff>=0 && ~departure_positions(i).taken_off
                        takeoff_count=takeoff_count+1; departure_positions(i).taken_off=true;
                    end
                    if time_to_exit<0, departure_positions(i).active=false; set(departure_markers(i),'Visible','off'); set(departure_labels(i),'Visible','off'); end
                else
                    departure_positions(i).active=false; set(departure_markers(i),'Visible','off'); set(departure_labels(i),'Visible','off');
                end
            end
        end
        
        % Kontrola separací
        min_sep_val_viz_dyn=Inf; conflict_detected_viz_dyn=false;
        for ac1=1:length(active_flights_now)
            for ac2=ac1+1:length(active_flights_now)
                dist_viz_dyn=haversine_distance(active_flights_now(ac1).lat,active_flights_now(ac1).lon,active_flights_now(ac2).lat,active_flights_now(ac2).lon);
                min_sep_val_viz_dyn=min(min_sep_val_viz_dyn,dist_viz_dyn);
                if dist_viz_dyn<MIN_SEPARATION_NM_AIR
                    conflict_detected_viz_dyn=true;
                    conflicts_detected=true;
                end
            end
        end
        
        if conflict_detected_viz_dyn
            separation_indicator.String=['KONFLIKT! Min.sep: ',num2str(min_sep_val_viz_dyn,'%.2f'),' NM'];
            separation_indicator.BackgroundColor='r';
        elseif min_sep_val_viz_dyn~=Inf
            separation_indicator.String=['Min.sep: ',num2str(min_sep_val_viz_dyn,'%.2f'),' NM'];
            separation_indicator.BackgroundColor='g';
        else
            separation_indicator.String='Žádné letouny';
            separation_indicator.BackgroundColor='g';
        end
        
        % Výpočet a zobrazení komplexní zátěže
        current_interval_idx = floor(current_time / WORKLOAD_INTERVAL_S) + 1;
        if current_interval_idx > length(current_dynamic_workload_profile)
            current_interval_idx = length(current_dynamic_workload_profile);
        end
        
        % Ukládání počtu aktivních letounů
        if current_interval_idx <= length(active_aircraft_count_per_interval)
            active_aircraft_count_per_interval(current_interval_idx) = length(active_flights_now);
        end
        
        % Získání komunikačních událostí pro aktuální interval
        current_comm_events = struct('time', {}, 'points', {}, 'flight_id', {}, 'duration', {});
        for k_comm = 1:length(aircraft_comm_events_for_dyn_sim)
            if aircraft_comm_events_for_dyn_sim(k_comm).time >= (current_interval_idx-1)*WORKLOAD_INTERVAL_S && ...
               aircraft_comm_events_for_dyn_sim(k_comm).time < current_interval_idx*WORKLOAD_INTERVAL_S
                current_comm_events(end+1) = aircraft_comm_events_for_dyn_sim(k_comm);
            end
        end
        comm_backlog = max(0, length(current_comm_events) - 2);
        
        % Výpočet komplexní zátěže
        if ~isempty(active_aircraft_positions)
            current_dynamic_workload_profile(current_interval_idx) = ...
                calculate_complete_workload_kbely(active_aircraft_positions, current_comm_events, ...
                                              current_interval_idx, conflicts_detected, comm_backlog, ...
                                              BASE_WORKLOAD_POINTS, WORKLOAD_INTERVAL_S);
        else
            current_dynamic_workload_profile(current_interval_idx) = BASE_WORKLOAD_POINTS;
        end
        
        % Zobrazení zátěže
        current_load_value_viz = current_dynamic_workload_profile(current_interval_idx);
        scale_percent = (current_load_value_viz / THEORETICAL_MAX_SCALE) * 100;
        if current_load_value_viz > OVERLOAD_THRESHOLD
            bg_color_wl_viz = [1 0.5 0.3]; status_str_wl_viz = 'PŘETÍŽENÍ';
        elseif current_load_value_viz > HIGH_WORKLOAD_THRESHOLD
            bg_color_wl_viz = [1 0.8 0.4]; status_str_wl_viz = 'Vysoká';
        elseif current_load_value_viz > MEDIUM_WORKLOAD_THRESHOLD
            bg_color_wl_viz = [0.85 1 0.7]; status_str_wl_viz = 'Střední';
        else
            bg_color_wl_viz = [0.7 1 0.7]; status_str_wl_viz = 'Nízká';
        end
        workload_indicator.String=sprintf('Zátěž: %.0f/300 (%.1f%%) - %s', current_load_value_viz, scale_percent, status_str_wl_viz);
        workload_indicator.BackgroundColor=bg_color_wl_viz;
        
        drawnow; pause(0.01);
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

% ----------------- 5. STATISTICKÉ VYHODNOCENÍ -------------------
disp(' ');
disp('--- Statistické vyhodnocení ---');
disp(['Celkem dokončeno: ', num2str(landing_count), ' přistání a ', num2str(takeoff_count), ' vzletů']);

% Detailní statistiky pohybů
total_movements = landing_count + takeoff_count;
arrivals_per_hour = landing_count / (SIM_DURATION_S / 3600);
departures_per_hour = takeoff_count / (SIM_DURATION_S / 3600);

disp(['Celkem pohybů: ', num2str(total_movements)]);
disp(['Příletů za hodinu: ', num2str(arrivals_per_hour, '%.2f')]);
disp(['Odletů za hodinu: ', num2str(departures_per_hour, '%.2f')]);

effective_hourly_capacity = (landing_count + takeoff_count) / (SIM_DURATION_S / 3600);
disp(['Efektivní hodinová kapacita: ', num2str(effective_hourly_capacity, '%.2f'), ' pohybů/hod']);

% Použijeme final_workload_per_interval z plánovací fáze pro finální graf,
% nebo current_dynamic_workload_profile pokud byla animace aktivní
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

fprintf('\n--- PARAMETRY SIMULACE ---\n');
fprintf('Vzdušná separace: %.0f NM\n', MIN_SEPARATION_NM_AIR);
fprintf('Střídavé plánování: ANO\n');

fprintf('\n=================================================\n');

% ----------------- GRAFICKÉ VÝSTUPY -------------------
if ~isempty(workload_data_for_graph) && any(workload_data_for_graph)
    figure('Name', 'Analýza zátěže řídícího - TMA Kbely (Střídavé plánování)', 'Position', [150, 150, 1100, 600]);
    time_minutes_wl = (0:length(workload_data_for_graph)-1) * (WORKLOAD_INTERVAL_S / 60);
    plot(time_minutes_wl, workload_data_for_graph, 'b-', 'LineWidth', 2);
    hold on;
    
    % ZJEDNODUŠENÉ PRAHOVÉ HODNOTY - pouze 3 čáry podle požadavku
    yline(OVERLOAD_THRESHOLD, '--r', ['Přetížení (>', num2str(OVERLOAD_THRESHOLD), ')'], 'Color', 'r', 'LineWidth', 2, 'LabelHorizontalAlignment', 'left','FontSize',12); 
    yline(HIGH_WORKLOAD_THRESHOLD, '--', ['Vysoká zátěž (', num2str(HIGH_WORKLOAD_THRESHOLD), '-', num2str(OVERLOAD_THRESHOLD), ')'], 'Color', [1, 0.7, 0], 'LineWidth', 2, 'LabelHorizontalAlignment', 'left','FontSize',12);
    yline(MEDIUM_WORKLOAD_THRESHOLD, '--g', ['Střední zátěž (', num2str(MEDIUM_WORKLOAD_THRESHOLD), '-', num2str(HIGH_WORKLOAD_THRESHOLD), ')'], 'LineWidth', 2, 'LabelHorizontalAlignment', 'left', 'Color', 'g', 'FontSize',12);

    xlabel('Čas (minuty)');
    ylabel('Zátěž řídícího (body)'); 
    title('Analýza pracovní zátěže řídícího - TMA Kbely (Střídavé plánování)'); 
    grid on;
    
    legend_handles = [plot(NaN,NaN,'b-','LineWidth',2), ...
                      line(NaN,NaN, 'LineStyle', '--', 'Color', 'g'), ...
                      line(NaN,NaN, 'LineStyle', '--', 'Color', [1, 0.7, 0]), ...
                      line(NaN,NaN, 'LineStyle', '--', 'Color', 'r')];
    legend(legend_handles, 'Vypočtená zátěž', 'Střední zátěž (90-162)', 'Vysoká zátěž (162-210)', 'Přetížení (>210)', 'Location', 'NorthEast');
    
    max_y_val_for_graph = max([max(workload_data_for_graph) * 1.1, OVERLOAD_THRESHOLD * 1.2, 100]);
    if isempty(max_y_val_for_graph) || isnan(max_y_val_for_graph) || max_y_val_for_graph == 0, max_y_val_for_graph = 250; end
    ylim([0, max_y_val_for_graph]);
    hold off;
end

% Graf korelace workload vs počet letounů
if ~isempty(workload_data_for_graph) && ~isempty(active_aircraft_count_per_interval) && ~isnan(correlation_value)
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
    
    xlabel('Čas (minuty)');
    title('Časový průběh počtu letounů a zátěže');
    grid on;
    legend('Počet letounů', 'Zátěž', 'Location', 'NorthEast');
end

% Časový diagram pohybů
figure('Name', 'Časový diagram pohybů (Střídavé plánování)', 'Position', [550, 550, 1000, 700]);
hold on;

% Vykreslení letů v pořadí jejich operací
y_pos = 0;
yticks_labels = {};

if ~isempty(scheduled_flights)
    for i = 1:length(scheduled_flights)
        flight = scheduled_flights(i);
        y_pos = i;
        
        if strcmp(flight.type, 'arrival')
            color_plot = 'b';
            if flight.route_index == 2, color_plot = 'g'; end
            
            plot([flight.entry_time, flight.landing_time], [y_pos, y_pos], '-', 'Color', color_plot, 'LineWidth', 3);
            plot(flight.entry_time, y_pos, 'o', 'Color', color_plot, 'MarkerSize', 6, 'MarkerFaceColor', color_plot);
            plot(flight.faf_time, y_pos, 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
            plot(flight.landing_time, y_pos, 'ks', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
            text(flight.entry_time - SIM_DURATION_S/60, y_pos, flight.id, 'FontSize', 8, 'HorizontalAlignment', 'right');
            yticks_labels{y_pos} = ['Přílet ', flight.id];
        else % departure
            plot([flight.takeoff_time, flight.exit_time], [y_pos, y_pos], '-m', 'LineWidth', 3);
            plot(flight.takeoff_time, y_pos, 'mo', 'MarkerSize', 6, 'MarkerFaceColor', 'm');
            plot(flight.vector1_time, y_pos, 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g');
            plot(flight.exit_time, y_pos, 'ms', 'MarkerSize', 6, 'MarkerFaceColor', 'm');
            text(flight.takeoff_time - SIM_DURATION_S/60, y_pos, flight.id, 'FontSize', 8, 'HorizontalAlignment', 'right');
            yticks_labels{y_pos} = ['Odlet ', flight.id];
        end
    end
end

xlabel('Čas (s)');
ylabel('Let (v pořadí plánování)');
title('Časový diagram pohybů - Střídavé plánování');
grid on;
xlim([0, SIM_DURATION_S]);

if ~isempty(yticks_labels)
    valid_yticks = find(~cellfun('isempty', yticks_labels));
    if ~isempty(valid_yticks)
        set(gca, 'YTick', valid_yticks, 'YTickLabel', yticks_labels(valid_yticks));
        ylim([0, max(valid_yticks) + 1]);
    end
end

legend_elements = [];
legend_texts = {};
legend_elements(end+1) = plot(NaN, NaN, '-bo', 'MarkerFaceColor','b', 'LineWidth',2); legend_texts{end+1} = 'Přílet (Trasa 1)';
legend_elements(end+1) = plot(NaN, NaN, '-go', 'MarkerFaceColor','g', 'LineWidth',2); legend_texts{end+1} = 'Přílet (Trasa 2)';
legend_elements(end+1) = plot(NaN, NaN, 'ro', 'MarkerFaceColor','r'); legend_texts{end+1} = 'FAF';
legend_elements(end+1) = plot(NaN, NaN, 'ks', 'MarkerFaceColor','k'); legend_texts{end+1} = 'Přistání';
legend_elements(end+1) = plot(NaN, NaN, '-mo', 'MarkerFaceColor','m', 'LineWidth',2); legend_texts{end+1} = 'Odlet';
legend_elements(end+1) = plot(NaN, NaN, 'go', 'MarkerFaceColor','g'); legend_texts{end+1} = 'VECTOR1';
legend_elements(end+1) = plot(NaN, NaN, 'ms', 'MarkerFaceColor','m'); legend_texts{end+1} = 'Opuštění (EXIT)';
legend(legend_elements, legend_texts, 'Location', 'bestoutside');
hold off;

% Uložení výsledků pro pozdější analýzu
disp(' ');
disp('--- Ukládání výsledků ---');
results = struct();
results.scheduled_flights = scheduled_flights;
results.rejected_flights = rejected_flights_info;
results.workload_profile = workload_data_for_graph;
results.active_aircraft_profile = active_aircraft_count_per_interval;
results.interleaved_planning = true;
results.separation_nm = MIN_SEPARATION_NM_AIR;
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

save_filename = sprintf('TMA_Kbely_results_interleaved_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
save(save_filename, 'results');
disp(['Výsledky uloženy do souboru: ', save_filename]);

disp(' ');
disp('Simulace TMA Kbely se střídavým plánováním úspěšně dokončena!');

% ----------------- POMOCNÉ FUNKCE -------------------

% --- FUNKCE PRO STŘÍDAVÉ PLÁNOVÁNÍ PRO KBELY (z Čáslavi) ---
function [planned_flights, rejected_flights_log, workload_profile, runway_ops_log] = ...
    plan_flights_with_interleaved_workload_control_kbely(orig_arrivals_t, orig_departures_t, ...
    arr_routes, dep_route, arr_route_prob, sim_dur_s, wl_interval_s, ...
    max_wl_points, base_wl_points, comm_params, min_sep_air_nm, ...
    rwy_occ_land_s, arr_arr_int_s, dep_arr_int_s, time_to_vector1_s)

    % 1. Vytvoření sjednoceného seznamu všech požadovaných pohybů
    all_movements_requests = [];
    for i = 1:length(orig_arrivals_t)
        movement.type = 'arrival';
        movement.original_time = orig_arrivals_t(i);
        movement.original_index = i;
        movement.route_index = select_route(arr_route_prob);
        movement.id = ['ARR', num2str(i)];
        all_movements_requests = [all_movements_requests; movement];
    end
    for i = 1:length(orig_departures_t)
        movement.type = 'departure';
        movement.original_time = orig_departures_t(i);
        movement.original_index = i;
        movement.route_index = 0; % Není třeba pro odlety
        movement.id = ['DEP', num2str(i)];
        all_movements_requests = [all_movements_requests; movement];
    end

    % Seřazení všech pohybů podle původního času
    if ~isempty(all_movements_requests)
        [~, sort_order] = sort([all_movements_requests.original_time]);
        all_movements_requests = all_movements_requests(sort_order);
    end

    % Inicializace
    planned_flights = struct('id', {}, 'type', {}, 'route_index', {}, ...
                           'entry_time', {}, 'faf_time', {}, 'landing_time', {}, ...
                           'takeoff_time', {}, 'vector1_time', {}, 'ekrot_time', {}, ...
                           'exit_time', {}, 'actual_operation_time', {}, ...
                           'taken_off', {}, 'landed', {});
    rejected_flights_log = [];
    runway_ops_log = struct('time', {}, 'type', {}, 'flight_id', {}, 'detail_time', {});
    
    num_wl_intervals = ceil(sim_dur_s / wl_interval_s);
    workload_profile = ones(1, num_wl_intervals) * base_wl_points;
    
    scheduled_comm_workload_points = struct('time', {}, 'points', {}, 'flight_id', {}, 'duration', {});

    MAX_PLANNING_DELAY_S = 600;
    TIME_STEP_PLANNING_S = 15;

    % 2. Hlavní plánovací smyčka přes všechny pohyby (STŘÍDAVĚ)
    for k = 1:length(all_movements_requests)
        current_movement_request = all_movements_requests(k);
        flight_planned_successfully = false;
        
        if strcmp(current_movement_request.type, 'arrival')
            % --- Plánování PŘÍLETU ---
            route = arr_routes{current_movement_request.route_index};
            orig_entry_time = current_movement_request.original_time;
            orig_faf_time = calculate_faf_arrival_time_kbely(current_movement_request.route_index, orig_entry_time, arr_routes);
            orig_landing_time = orig_entry_time + route.total_time;

            if orig_landing_time > sim_dur_s, continue; end

            for delay_s = 0:TIME_STEP_PLANNING_S:MAX_PLANNING_DELAY_S
                current_flight_candidate = struct();
                current_flight_candidate.id = [current_movement_request.id, '_d', num2str(delay_s), '_'];
                current_flight_candidate.type = 'arrival';
                current_flight_candidate.route_index = current_movement_request.route_index;
                current_flight_candidate.entry_time = orig_entry_time + delay_s;
                current_flight_candidate.faf_time = orig_faf_time + delay_s;
                current_flight_candidate.landing_time = orig_landing_time + delay_s;
                current_flight_candidate.actual_operation_time = current_flight_candidate.landing_time;
                current_flight_candidate.takeoff_time = NaN;
                current_flight_candidate.vector1_time = NaN;
                current_flight_candidate.ekrot_time = NaN;
                current_flight_candidate.exit_time = NaN;
                current_flight_candidate.taken_off = false;
                current_flight_candidate.landed = false;

                if current_flight_candidate.landing_time > sim_dur_s, continue; end

                % Komunikační události
                temp_comm_events_for_this_flight = struct('time', {}, 'points', {}, 'flight_id', {}, 'duration', {});
                temp_comm_events_for_this_flight(end+1) = struct('time', current_flight_candidate.entry_time + comm_params.initial_contact.timing_offset_s, 'points', comm_params.initial_contact.controller_workload_points, 'flight_id', current_flight_candidate.id, 'duration', comm_params.initial_contact.duration_total_s);
                temp_comm_events_for_this_flight(end+1) = struct('time', current_flight_candidate.faf_time + comm_params.faf_report.timing_offset_s, 'points', comm_params.faf_report.controller_workload_points, 'flight_id', current_flight_candidate.id, 'duration', comm_params.faf_report.duration_total_s);

                % A. Kontrola dostupnosti dráhy
                runway_sep_ok = check_runway_separation_rules_kbely(current_flight_candidate, runway_ops_log, ...
                                                                rwy_occ_land_s, arr_arr_int_s, dep_arr_int_s, time_to_vector1_s);
                if ~runway_sep_ok, continue; end

                % B. Kontrola vzdušné separace s existujícími lety
                air_sep_ok = true;
                if ~isempty(planned_flights)
                    air_sep_ok = check_all_air_separations_kbely(current_flight_candidate, planned_flights, ...
                        arr_routes, dep_route, min_sep_air_nm, time_to_vector1_s);
                end
                if ~air_sep_ok, continue; end

                % C. Kontrola workloadu
                [workload_ok, temp_workload_profile] = check_complex_workload_impact_kbely(...
                    current_flight_candidate, temp_comm_events_for_this_flight, ...
                    workload_profile, scheduled_comm_workload_points, ...
                    num_wl_intervals, wl_interval_s, base_wl_points, max_wl_points, ...
                    planned_flights, arr_routes, dep_route, time_to_vector1_s);
                if ~workload_ok, continue; end

                % Pokud všechny kontroly prošly
                planned_flights(end+1) = current_flight_candidate;
                workload_profile = temp_workload_profile;
                scheduled_comm_workload_points = [scheduled_comm_workload_points, temp_comm_events_for_this_flight];
                if ~isempty(scheduled_comm_workload_points)
                    [~, sort_comm_idx] = sort([scheduled_comm_workload_points.time]);
                    scheduled_comm_workload_points = scheduled_comm_workload_points(sort_comm_idx);
                end

                new_op_log = struct('time', current_flight_candidate.actual_operation_time, 'type', current_flight_candidate.type, 'flight_id', current_flight_candidate.id);
                new_op_log.detail_time = current_flight_candidate.landing_time;
                runway_ops_log = [runway_ops_log, new_op_log];
                if ~isempty(runway_ops_log)
                    [~, sort_rwy_idx] = sort([runway_ops_log.time]);
                    runway_ops_log = runway_ops_log(sort_rwy_idx);
                end

                flight_planned_successfully = true;
                break;
            end

        elseif strcmp(current_movement_request.type, 'departure')
            % --- Plánování ODLETU ---
            orig_takeoff_time = current_movement_request.original_time;
            [orig_vector1_time, orig_ekrot_time, orig_exit_time] = calculate_departure_times_kbely(orig_takeoff_time, dep_route);

            if orig_exit_time > sim_dur_s, continue; end

            for delay_s = 0:TIME_STEP_PLANNING_S:MAX_PLANNING_DELAY_S
                current_flight_candidate = struct();
                current_flight_candidate.id = [current_movement_request.id, '_d', num2str(delay_s), '_'];
                current_flight_candidate.type = 'departure';
                current_flight_candidate.takeoff_time = orig_takeoff_time + delay_s;
                current_flight_candidate.vector1_time = orig_vector1_time + delay_s;
                current_flight_candidate.ekrot_time = orig_ekrot_time + delay_s;
                current_flight_candidate.exit_time = orig_exit_time + delay_s;
                current_flight_candidate.actual_operation_time = current_flight_candidate.takeoff_time;
                current_flight_candidate.route_index = NaN;
                current_flight_candidate.entry_time = NaN;
                current_flight_candidate.faf_time = NaN;
                current_flight_candidate.landing_time = NaN;
                current_flight_candidate.taken_off = false;
                current_flight_candidate.landed = false;

                if current_flight_candidate.exit_time > sim_dur_s, continue; end

                % Komunikační události
                temp_comm_events_for_this_flight = struct('time', {}, 'points', {}, 'flight_id', {}, 'duration', {});
                temp_comm_events_for_this_flight(end+1) = struct('time', current_flight_candidate.takeoff_time + comm_params.takeoff_clearance.timing_offset_s, 'points', comm_params.takeoff_clearance.controller_workload_points, 'flight_id', current_flight_candidate.id, 'duration', comm_params.takeoff_clearance.duration_total_s);
                temp_comm_events_for_this_flight(end+1) = struct('time', current_flight_candidate.vector1_time + comm_params.departure_contact.timing_offset_s, 'points', comm_params.departure_contact.controller_workload_points, 'flight_id', current_flight_candidate.id, 'duration', comm_params.departure_contact.duration_total_s);
                temp_comm_events_for_this_flight(end+1) = struct('time', current_flight_candidate.exit_time + comm_params.exit_clearance.timing_offset_s, 'points', comm_params.exit_clearance.controller_workload_points, 'flight_id', current_flight_candidate.id, 'duration', comm_params.exit_clearance.duration_total_s);

                % A. Kontrola dostupnosti dráhy
                runway_sep_ok = check_runway_separation_rules_kbely(current_flight_candidate, runway_ops_log, ...
                                                                rwy_occ_land_s, arr_arr_int_s, dep_arr_int_s, time_to_vector1_s);
                if ~runway_sep_ok, continue; end

                % B. Kontrola vzdušné separace s existujícími lety
                air_sep_ok = true;
                if ~isempty(planned_flights)
                    air_sep_ok = check_all_air_separations_kbely(current_flight_candidate, planned_flights, ...
                        arr_routes, dep_route, min_sep_air_nm, time_to_vector1_s);
                end
                if ~air_sep_ok, continue; end

                % C. Kontrola workloadu
                [workload_ok, temp_workload_profile] = check_complex_workload_impact_kbely(...
                    current_flight_candidate, temp_comm_events_for_this_flight, ...
                    workload_profile, scheduled_comm_workload_points, ...
                    num_wl_intervals, wl_interval_s, base_wl_points, max_wl_points, ...
                    planned_flights, arr_routes, dep_route, time_to_vector1_s);
                if ~workload_ok, continue; end

                % Pokud všechny kontroly prošly
                planned_flights(end+1) = current_flight_candidate;
                workload_profile = temp_workload_profile;
                scheduled_comm_workload_points = [scheduled_comm_workload_points, temp_comm_events_for_this_flight];
                if ~isempty(scheduled_comm_workload_points)
                    [~, sort_comm_idx] = sort([scheduled_comm_workload_points.time]);
                    scheduled_comm_workload_points = scheduled_comm_workload_points(sort_comm_idx);
                end

                new_op_log = struct('time', current_flight_candidate.actual_operation_time, 'type', current_flight_candidate.type, 'flight_id', current_flight_candidate.id);
                new_op_log.detail_time = current_flight_candidate.vector1_time;
                runway_ops_log = [runway_ops_log, new_op_log];
                if ~isempty(runway_ops_log)
                    [~, sort_rwy_idx] = sort([runway_ops_log.time]);
                    runway_ops_log = runway_ops_log(sort_rwy_idx);
                end

                flight_planned_successfully = true;
                break;
            end
        end

        if ~flight_planned_successfully
            rejected_flights_log(end+1).id = current_movement_request.id;
            rejected_flights_log(end).type = current_movement_request.type;
            rejected_flights_log(end).orig_time = current_movement_request.original_time;
            rejected_flights_log(end).reason = 'Nenalezen slot splňující separace a limit zátěže (střídavé plánování).';
        end
    end

    % Výsledky již jsou v pořadí plánování (střídavě)
end

% --- Komplexní model zátěže (převzato z Čáslavi) ---
function controller_workload = calculate_complete_workload_kbely(active_aircraft_positions, comm_events, current_minute, conflicts_detected, comm_backlog, base_workload, interval_duration_s)
    
    total_active_aircraft = size(active_aircraft_positions, 1);
    
    % 1. Základní zátěž
    % base_workload je předán jako parametr
    
    % 2. Monitoring zátěž
    basic_tracking = total_active_aircraft * 2;
    
    num_aircraft_pairs = 0;
    if total_active_aircraft >= 2
        num_aircraft_pairs = total_active_aircraft * (total_active_aircraft - 1) / 2;
    end
    separation_monitoring = num_aircraft_pairs * 0.5;
    
    prediction_base = total_active_aircraft * 1;
    if total_active_aircraft > 3
        complexity_factor = (total_active_aircraft - 3) * 0.5;
        prediction_workload = prediction_base * (1 + complexity_factor);
    else
        prediction_workload = prediction_base;
    end
    
    monitoring_workload = basic_tracking + separation_monitoring + prediction_workload;
    
    % 3. Komunikační zátěž
    comm_workload_val = 0;
    minute_start = (current_minute - 1) * interval_duration_s;
    minute_end = current_minute * interval_duration_s;
    
    for e_idx = 1:length(comm_events)
        if comm_events(e_idx).time >= minute_start && comm_events(e_idx).time < minute_end
            comm_workload_val = comm_workload_val + comm_events(e_idx).points * (comm_events(e_idx).duration / interval_duration_s);
        end
    end
    
    % 4. Konfliktní zátěž
    conflict_workload_val = 0;
    for ac1 = 1:size(active_aircraft_positions, 1)
        for ac2 = ac1+1:size(active_aircraft_positions, 1)
            distance = haversine_distance(active_aircraft_positions(ac1,1), active_aircraft_positions(ac1,2), ...
                                        active_aircraft_positions(ac2,1), active_aircraft_positions(ac2,2));
            if distance < 10 && distance >= 7
                conflict_workload_val = conflict_workload_val + 5;
            elseif distance < 7 && distance >= 5
                conflict_workload_val = conflict_workload_val + 10;
            elseif distance < 5
                conflict_workload_val = conflict_workload_val + 20;
            end
        end
    end
    
    % 5. Kognitivní zátěž
    cognitive_load = 10;
    if total_active_aircraft > 4
        cognitive_multiplier = 1 + (total_active_aircraft - 4) * 0.10;
        cognitive_load = cognitive_load * cognitive_multiplier;
    end
    
    % Stresové faktory
    if conflicts_detected
        cognitive_load = cognitive_load + 20;
    end
    if comm_backlog > 2
        cognitive_load = cognitive_load + 10;
    end
    
    % Celková zátěž
    controller_workload = base_workload + monitoring_workload + comm_workload_val + conflict_workload_val + cognitive_load;
end

% --- Kontrola komplexní zátěže ---
function [workload_is_ok, new_workload_profile] = check_complex_workload_impact_kbely(...
    candidate_flight, candidate_comm_events, ...
    current_workload_profile, existing_comm_events, ...
    num_intervals, interval_s, base_wl, max_wl, all_currently_planned_flights, ...
    arr_routes, dep_route, time_to_vector1)

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
        
        % Získání aktivních letadel
        active_aircraft_positions = [];
        conflicts_detected = false;
        comm_backlog = 0;
        
        for k_fl = 1:length(temp_all_planned_flights)
            fl_obj = temp_all_planned_flights(k_fl);
            fl_s_time = 0; fl_e_time = 0;
            if strcmp(fl_obj.type, 'arrival')
                fl_s_time = fl_obj.entry_time; fl_e_time = fl_obj.landing_time;
            else
                fl_s_time = fl_obj.takeoff_time; fl_e_time = fl_obj.exit_time;
            end
            
            if max(fl_s_time, interval_start_t) < min(fl_e_time, interval_end_t)
                mid_interval_time = (interval_start_t + interval_end_t) / 2;
                if mid_interval_time >= fl_s_time && mid_interval_time <= fl_e_time
                    pos = get_flight_position_at_time_kbely(fl_obj, mid_interval_time, arr_routes, dep_route, time_to_vector1);
                    if ~isempty(pos)
                        active_aircraft_positions(end+1,:) = [pos.lat, pos.lon];
                    end
                end
            end
        end
        
        % Kontrola konfliktů
        for ac1 = 1:size(active_aircraft_positions, 1)
            for ac2 = ac1+1:size(active_aircraft_positions, 1)
                distance = haversine_distance(active_aircraft_positions(ac1,1), active_aircraft_positions(ac1,2), ...
                                            active_aircraft_positions(ac2,1), active_aircraft_positions(ac2,2));
                if distance < 5
                    conflicts_detected = true;
                end
            end
        end
        
        % Získání komunikačních událostí
        current_comm_events = struct('time', {}, 'points', {}, 'flight_id', {}, 'duration', {});
        for k_c_ev = 1:length(temp_all_comm_events)
            c_ev = temp_all_comm_events(k_c_ev);
            if c_ev.time >= interval_start_t && c_ev.time < interval_end_t
                current_comm_events(end+1) = c_ev;
            end
        end
        comm_backlog = max(0, length(current_comm_events) - 2);

        % Výpočet komplexní zátěže
        new_total_wl_for_interval_i = calculate_complete_workload_kbely(...
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

% --- Kontrola separace na dráze (pokročilá pravidla) ---
function is_ok = check_runway_separation_rules_kbely(candidate_flight, rwy_log, rwy_occ_after_land_s, arr_arr_sep_s, dep_arr_sep_s, ~)
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
            % Kontrola A1-A2
            if cand_landing_time < (last_op.detail_time + arr_arr_sep_s)
                is_ok = false; return;
            end
            if cand_landing_time < (last_op.detail_time + rwy_occ_after_land_s)
                 is_ok = false; return;
            end
        elseif strcmp(last_op.type, 'departure')
            % Kontrola A2 po D1
            if cand_landing_time < last_op.detail_time
                is_ok = false; return;
            end
            % Speciální pravidlo A1-D1-A2
            idx_A1_candidates = find(strcmp({rwy_log.type},'arrival') & [rwy_log.time] < last_op.time);
            if ~isempty(idx_A1_candidates)
                [~, closest_A1_idx_in_candidates] = max([rwy_log(idx_A1_candidates).time]);
                A1_landing_time = rwy_log(idx_A1_candidates(closest_A1_idx_in_candidates)).detail_time;
                if cand_landing_time < (A1_landing_time + 180) % 3 minuty
                    is_ok = false; return;
                end
            end
        end
    else % departure
        cand_takeoff_time = candidate_flight.takeoff_time;

        if strcmp(last_op.type, 'arrival')
            % Kontrola D1 po A1
            if cand_takeoff_time < (last_op.detail_time + dep_arr_sep_s) || cand_takeoff_time < (last_op.detail_time + rwy_occ_after_land_s)
                is_ok = false; return;
            end
        elseif strcmp(last_op.type, 'departure')
            % Kontrola D1-D2
            if cand_takeoff_time < last_op.detail_time
                is_ok = false; return;
            end
        end
    end
end

% --- Kontrola vzdušné separace ---
function all_sep_ok = check_all_air_separations_kbely(candidate_flight, existing_flights, arr_routes, dep_route, min_sep_nm, time_to_vector1)
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
            pos_cand = get_flight_position_at_time_kbely(candidate_flight, t_check, arr_routes, dep_route, time_to_vector1);
            pos_exist = get_flight_position_at_time_kbely(existing_flight, t_check, arr_routes, dep_route, time_to_vector1);

            if isempty(pos_cand) || isempty(pos_exist), continue; end

            dist_nm = haversine_distance(pos_cand.lat, pos_cand.lon, pos_exist.lat, pos_exist.lon);
            if dist_nm < min_sep_nm
                all_sep_ok = false;
                return;
            end
        end
    end
end

% --- Získání pozice letadla v daném čase ---
function position = get_flight_position_at_time_kbely(flight, abs_time, arr_routes, dep_route, time_to_vector1)
    position = [];
    if strcmp(flight.type, 'arrival')
        if abs_time >= flight.entry_time && abs_time <= flight.landing_time
            time_in_route = abs_time - flight.entry_time;
            route_details = arr_routes{flight.route_index};
            dist_traveled = calculate_distance_traveled_kbely(flight.route_index, time_in_route, arr_routes);
            [lat, lon, speed] = interpolate_position_kbely(route_details, dist_traveled);
            position.lat = lat; position.lon = lon; position.speed = speed;
        end
    else
        if abs_time >= flight.takeoff_time && abs_time <= flight.exit_time
            time_since_takeoff = abs_time - flight.takeoff_time;
            dist_traveled = calculate_distance_traveled_departure_kbely(time_since_takeoff, dep_route);
            [lat, lon, speed] = interpolate_position_departure_kbely(dep_route, dist_traveled);
            position.lat = lat; position.lon = lon; position.speed = speed;
        end
    end
end

% --- Pomocné funkce pro výpočty pozic a vzdáleností (zachováno z Kbely) ---
function dist_traveled = calculate_distance_traveled_kbely(route_idx, time_in_route, all_routes_cell)
    route_obj = all_routes_cell{route_idx};
    if time_in_route <= 0, dist_traveled = 0; return; end
    if isempty(route_obj.segment_time), dist_traveled = 0; return; end
    
    seg_t = route_obj.segment_time;
    seg_d = route_obj.segment_dist;
    acc_t = 0; acc_d = 0;
    
    for i_seg = 1:length(seg_t)
        seg_end_t = acc_t + seg_t(i_seg);
        if time_in_route <= seg_end_t + eps || i_seg == length(seg_t)
            if seg_t(i_seg) == 0, fr = 0;
            else, fr = min(1, max(0, (time_in_route - acc_t) / seg_t(i_seg))); end
            dist_traveled = acc_d + fr * seg_d(i_seg);
            return;
        end
        acc_t = seg_end_t;
        acc_d = acc_d + seg_d(i_seg);
    end
    dist_traveled = acc_d;
end

function [lat, lon, speed] = interpolate_position_kbely(route_obj, dist_traveled_val)
    segment_start_dist = 0; segment_idx = 0;
    
    if isempty(route_obj.segment_dist) || route_obj.total_dist == 0
        wp1_c = route_obj.waypoint_coords(1,:);
        lat = wp1_c(1); lon = wp1_c(2);
        speed = route_obj.speed_at_waypoints(1);
        return;
    end
    
    if dist_traveled_val <= 0
        segment_idx = 1; dist_traveled_val = 0;
    elseif dist_traveled_val >= route_obj.total_dist
        segment_idx = length(route_obj.segment_dist);
        dist_traveled_val = route_obj.total_dist;
    else
        for i_seg = 1:length(route_obj.segment_dist)
            segment_end_dist = segment_start_dist + route_obj.segment_dist(i_seg);
            if dist_traveled_val <= segment_end_dist + eps
                segment_idx = i_seg; break;
            end
            segment_start_dist = segment_end_dist;
        end
    end
    
    if segment_idx == 0 && ~isempty(route_obj.segment_dist)
        segment_idx = length(route_obj.segment_dist);
    elseif segment_idx == 0
        segment_idx = 1;
    end
    
    if route_obj.segment_dist(segment_idx) == 0, seg_rel_pos = 0;
    else, seg_rel_pos = (dist_traveled_val - segment_start_dist) / route_obj.segment_dist(segment_idx); end
    seg_rel_pos = min(1, max(0, seg_rel_pos));
    
    wp1_c = route_obj.waypoint_coords(segment_idx,:);
    wp2_c = route_obj.waypoint_coords(segment_idx + 1,:);
    lat = wp1_c(1) + seg_rel_pos * (wp2_c(1) - wp1_c(1));
    lon = wp1_c(2) + seg_rel_pos * (wp2_c(2) - wp1_c(2));
    
    sp1 = route_obj.speed_at_waypoints(segment_idx);
    sp2 = route_obj.speed_at_waypoints(segment_idx + 1);
    speed = sp1 + seg_rel_pos * (sp2 - sp1);
end

function dist_traveled = calculate_distance_traveled_departure_kbely(time_since_takeoff, route_dep_details_obj)
    if time_since_takeoff <= 0, dist_traveled = 0; return; end
    if isempty(route_dep_details_obj.segment_time), dist_traveled = 0; return; end
    
    segment_times_dep = route_dep_details_obj.segment_time;
    segment_distances_dep = route_dep_details_obj.segment_dist;
    accumulated_time_dep = 0; accumulated_distance_dep = 0;
    
    for i = 1:length(segment_times_dep)
        segment_end_time_dep = accumulated_time_dep + segment_times_dep(i);
        if time_since_takeoff <= segment_end_time_dep + eps || i == length(segment_times_dep)
            if segment_times_dep(i) == 0, frac_dep = 0;
            else, frac_dep = min(1, max(0, (time_since_takeoff - accumulated_time_dep) / segment_times_dep(i))); end
            dist_traveled = accumulated_distance_dep + frac_dep * segment_distances_dep(i);
            return;
        end
        accumulated_time_dep = segment_end_time_dep;
        accumulated_distance_dep = accumulated_distance_dep + segment_distances_dep(i);
    end
    dist_traveled = accumulated_distance_dep;
end

function [lat, lon, speed] = interpolate_position_departure_kbely(route_dep_details_obj, dist_traveled_val)
    segment_start_dist_dep = 0; segment_idx_dep = 0;
    
    if isempty(route_dep_details_obj.segment_dist) || route_dep_details_obj.total_dist == 0
        wp1_c_d = route_dep_details_obj.waypoint_coords(1,:);
        lat = wp1_c_d(1); lon = wp1_c_d(2);
        speed = route_dep_details_obj.speed_at_waypoints(1);
        return;
    end
    
    if dist_traveled_val <= 0
        segment_idx_dep = 1; dist_traveled_val = 0;
    elseif dist_traveled_val >= route_dep_details_obj.total_dist
        segment_idx_dep = length(route_dep_details_obj.segment_dist);
        dist_traveled_val = route_dep_details_obj.total_dist;
    else
        for i_s = 1:length(route_dep_details_obj.segment_dist)
            segment_end_dist_dep = segment_start_dist_dep + route_dep_details_obj.segment_dist(i_s);
            if dist_traveled_val <= segment_end_dist_dep + eps
                segment_idx_dep = i_s; break;
            end
            segment_start_dist_dep = segment_end_dist_dep;
        end
    end
    
    if segment_idx_dep == 0 && ~isempty(route_dep_details_obj.segment_dist)
        segment_idx_dep = length(route_dep_details_obj.segment_dist);
    elseif segment_idx_dep == 0
        segment_idx_dep = 1;
    end
    
    if route_dep_details_obj.segment_dist(segment_idx_dep) == 0, seg_rel_pos_dep = 0;
    else, seg_rel_pos_dep = (dist_traveled_val - segment_start_dist_dep) / route_dep_details_obj.segment_dist(segment_idx_dep); end
    seg_rel_pos_dep = min(1, max(0, seg_rel_pos_dep));
    
    wp1_c_d = route_dep_details_obj.waypoint_coords(segment_idx_dep,:);
    wp2_c_d = route_dep_details_obj.waypoint_coords(segment_idx_dep + 1,:);
    lat = wp1_c_d(1) + seg_rel_pos_dep * (wp2_c_d(1) - wp1_c_d(1));
    lon = wp1_c_d(2) + seg_rel_pos_dep * (wp2_c_d(2) - wp1_c_d(2));
    
    sp1_d = route_dep_details_obj.speed_at_waypoints(segment_idx_dep);
    sp2_d = route_dep_details_obj.speed_at_waypoints(segment_idx_dep + 1);
    speed = sp1_d + seg_rel_pos_dep * (sp2_d - sp1_d);
end

% --- Další pomocné funkce ---
function faf_time_calc = calculate_faf_arrival_time_kbely(route_idx_local, entry_time_local, all_arrival_routes_local)
    current_route_obj = all_arrival_routes_local{route_idx_local};
    time_to_faf = 0; faf_reached_flag = false;
    
    for seg_idx_faf = 1:length(current_route_obj.waypoints) - 1
        time_to_faf = time_to_faf + current_route_obj.segment_time(seg_idx_faf);
        if strcmp(current_route_obj.waypoints{seg_idx_faf + 1}, 'FAF')
            faf_reached_flag = true; break;
        end
    end
    
    if faf_reached_flag
        faf_time_calc = entry_time_local + time_to_faf;
    else
        faf_time_calc = entry_time_local + current_route_obj.total_time;
    end
end

function [vector1_t_calc, ekrot_t_calc, exit_pt_t_calc] = calculate_departure_times_kbely(dep_time_loc, dep_route_obj_loc)
    vector1_t_calc = dep_time_loc; ekrot_t_calc = dep_time_loc; exit_pt_t_calc = dep_time_loc;
    seg_times_dep = dep_route_obj_loc.segment_time;
    
    if length(seg_times_dep) >= 1
        vector1_t_calc = dep_time_loc + seg_times_dep(1);
        ekrot_t_calc = vector1_t_calc;
        exit_pt_t_calc = vector1_t_calc;
    end
    if length(seg_times_dep) >= 2
        ekrot_t_calc = vector1_t_calc + seg_times_dep(2);
        exit_pt_t_calc = ekrot_t_calc;
    end
    if length(seg_times_dep) >= 3
        exit_pt_t_calc = ekrot_t_calc + seg_times_dep(3);
    end
end

function decimal_degrees = custom_dms2degrees(dms_str)
    if contains(dms_str,'N') || contains(dms_str,'S')
        is_north = contains(dms_str,'N');
        dms_str_cleaned = strrep(strrep(dms_str,'N',''),'S','');
        degrees = str2double(dms_str_cleaned(1:2));
        minutes = str2double(dms_str_cleaned(3:4));
        seconds = str2double(dms_str_cleaned(5:end));
        decimal_degrees = degrees + minutes/60 + seconds/3600;
        if ~is_north, decimal_degrees = -decimal_degrees; end
    elseif contains(dms_str,'E') || contains(dms_str,'W')
        is_east = contains(dms_str,'E');
        dms_str_cleaned = strrep(strrep(dms_str,'E',''),'W','');
        degrees = str2double(dms_str_cleaned(1:3));
        minutes = str2double(dms_str_cleaned(4:5));
        seconds = str2double(dms_str_cleaned(6:end));
        decimal_degrees = degrees + minutes/60 + seconds/3600;
        if ~is_east, decimal_degrees = -decimal_degrees; end
    else
        error('Neznámý formát DMS: %s', dms_str);
    end
end

function d = haversine_distance(lat1, lon1, lat2, lon2)
    lat1_rad = lat1 * pi/180; lon1_rad = lon1 * pi/180;
    lat2_rad = lat2 * pi/180; lon2_rad = lon2 * pi/180;
    R_earth_nm = 3440.065;
    a = sin((lat2_rad - lat1_rad)/2)^2 + cos(lat1_rad) * cos(lat2_rad) * sin((lon2_rad - lon1_rad)/2)^2;
    c = 2 * atan2(sqrt(a), sqrt(1 - a));
    d = R_earth_nm * c;
end

function [course_deg] = calculate_initial_course(lat1, lon1, lat2, lon2)
    lat1_rad = lat1 * pi/180; lon1_rad = lon1 * pi/180;
    lat2_rad = lat2 * pi/180; lon2_rad = lon2 * pi/180;
    dlon_rad = lon2_rad - lon1_rad;
    y = sin(dlon_rad) * cos(lat2_rad);
    x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dlon_rad);
    course_rad = atan2(y, x);
    course_deg = mod(course_rad * 180/pi + 360, 360);
end

function route_idx = select_route(probabilities_arr)
    r_val = rand(); cum_prob = 0;
    for idx = 1:length(probabilities_arr)
        cum_prob = cum_prob + probabilities_arr(idx);
        if r_val <= cum_prob, route_idx = idx; return; end
    end
    route_idx = length(probabilities_arr);
end

function arrival_times = generate_uniform_arrivals(num_ac, max_t, buf_t)
    if num_ac == 0, arrival_times = []; return; end
    usable_t = max_t - buf_t;
    if usable_t <= 0, arrival_times = zeros(1, num_ac) + buf_t/2; return; end
    intvl = usable_t / num_ac;
    if num_ac == 1, arrival_times = usable_t/2;
    else, arrival_times = (0:num_ac - 1) * intvl; end
    arrival_times = arrival_times + (rand(1, num_ac) - 0.5) * 0.1 * intvl;
    arrival_times = sort(max(0, min(usable_t, arrival_times)));
end

function departure_times = generate_uniform_departures(num_dep, max_t, buf_t)
    if num_dep == 0, departure_times = []; return; end
    usable_t = max_t - buf_t;
    if usable_t <= 0, departure_times = zeros(1, num_dep) + buf_t/2; return; end
    intvl = usable_t / num_dep;
    if num_dep == 1, departure_times = usable_t/2;
    else, departure_times = (0:num_dep - 1) * intvl; end
    departure_times = departure_times + (rand(1, num_dep) - 0.5) * 0.1 * intvl;
    departure_times = sort(max(0, min(usable_t, departure_times)));
end