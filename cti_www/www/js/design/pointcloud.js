var project = "";

var line2 = [];
var line3 = [];
var line5=[];

var log = "none";
var log_sum = "";

var switch_submit_data = 0;
var is_line2orline3 = -1;
var is_roadedgeorcleararea = 1;

var container, stats;
var camera, scene, renderer, controls;
var mouseX = 0, mouseY = 0;

var init_line2 = 0;
var init_line3 = 0;
var flag_2dOr3d_switch = 0;
var if_filter = 1;
var numb_update_line = 100;
var if_exist_cleareare = 0;

var high_light_point_for_line = -1;
var create_newpoint_live = 0;
var record_i;

var pose_x;
var pose_y;
var pose_z = 0;
var yaw;
var person_view_x;
var person_view_y;
var person_view_z;

var pub_once_rs = 0;
var rs_count = 0;

var pub_once_rs_bpe = 0;
var rs_count_bpe = 0;

var pub_once_rs_lh_back = 0;
var rs_count_lh_back = 0;

var pub_once_rs_mid_back = 0;
var rs_count_mid_back = 0;

var pub_once_rs_livox = 0;
var rs_count_livox = 0;


var points_rs_bpe;

pointsArray = new Array();
pointslineArray = new Array();
line_object_Array = new Array();




var position_rs = [];
var mesh_rs;

var position_rs_bpe = [];
var mesh_rs_bpe;

var position_lh_back = [];
var mesh_lh_back;

var position_livox = [];
var mesh_livox;

var position_mid_scan = [];
var mesh_mid_scan;

var foreview = 0; 
var leftview = 0;
var verticleview = 1;
var anotherview = 0;

var lock1 = 0;
var lock2 = 0;
var lock3 = 1;
var lock4 = 0;

var calibration_version = 0;
var calibration_lidar = 0;

var start_calibrate_flag = 0;

var luanch_numb = "7";
var switch_launch = 0;

var calibrating_flag = 0;

var switch_on_shell_once = 0;

var close_calibration_node = 0;

var once_close_lidar = 0 ;

var request_tf_diary = 0;




var calibrate_success_count = 0;

var calibrate_success_front_flag = 0;

var calibrate_success_rear_flag = 0;

var calibrate_success_mid_single_flag = 0;

var calibrate_success_rear_single_front_flag = 0;