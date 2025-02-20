
  // Connecting to ROS
  // -----------------

  var ros = new ROSLIB.Ros({
    url : `ws://${window.location.hostname}:8082`
  });

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
    show_health_status(0);
    doresize();
  });

  ros.on('error', function(error) {
    console.log(`Error connecting to websocket server @${window.location.hostname}: ${error}`);
    show_health_status(0);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
    show_health_status(0);
  });

  function doresize() {
    var header = document.getElementById("header");
    var footer = document.getElementById("footer");
    var body = document.getElementById("main_body");
    var page = document.getElementById("page");

    header_size = header.getBoundingClientRect();
    footer_size = footer.getBoundingClientRect();
    page_size = footer_size.top - header_size.bottom-30;

    body.style.height = `${(page_size).toFixed(0)}px`;
    console.log(`header: ${header_size.height}, footer: ${footer_size.height}, page: ${page_size.height}, new body: ${body.style.height}`);
  };
  
  var quori_ivt = new ROSLIB.Message({
    num_values : 0 ,   // number of valid entries in the arrays
    id : [0],  // from IVT_ID above
    value  : [0],  // Corresponding measurement value
    valid  : [0]     // Corresponding measurement value is valid if true
  });

  // Subscribing to a Topic
  // ----------------------

  var ivt_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/quori_ivt',
    messageType : 'quori_msgs/QuoriIVT'
  });

  ivt_listener.subscribe(function(message) {
    console.log('Received message on ' + ivt_listener.name + ': ' + toString(message));

    var ivt=document.getElementById('ivt');
    var txt = "<table><caption><b>IVT</b></caption><tr><th>Sensor</th><th>Value</th><th>Units</th></tr>";

    for(v=0; v < message.num_values; v++) {

      var name="Unknown";
      var units="unknown";

      switch(message.ivt_id[v])  {

          case 1:
            name="HEAD STM TEMP";
            units="&deg;C";
            break;

          case 2:
            name="BASE STM TEMP";
            units="&deg;C";
            break;

          case 4:
            name="MAIN STM TEMP";
            units="&deg;C";
            break;

          case 8: 
            name="POWER STM TEMP";
            units="&deg;C";
            break;

          case 16: 
            name="RBTQ VMOT";
            units="Volts";
            break;

          case 32: 
            name= "24V BUS CURRENT";
            units="Amps";
            break;

          case 64: 
            name="12V BUS CURRENT";
            units="Amps";
            break;

          case 128: 
            name="5V EXTRA CURRENT";
            units="Amps";
            break;

          case 256: 
            name="ORIN CURRENT";
            units="Amps";
            break;

          case 512: 
            name="POWER BOARD 24V BUS CURRENT";
            units="Amps";
            break;

          case 1024: 
            name="POWER BOARD 12V BUS CURRENT";
            units="Amps";
            break;

          case 2048: 
            name="BATT VOLTS";
            units="Volts";
            break;

          case 4096:
            name="BATT CURRENT";
            units="Amps";
            break;

          case 8192: 
            name="BATT SOC";
            units="%";
            bat_ind = document.getElementById("batt_ind");
            bat_ind.style.width =  message.value[v].toFixed(0) + "%";
            break;
      }
      if(message.valid[v]) {
        txt += "<tr><td>" + name + "</td><td>" + message.value[v].toFixed(2) + "</td><td>" + units + "</td></tr>";
      }
      else {
        txt += "<tr><td>" + name + "</td><td>unknown</td><td>unknown</td></tr>";
      }
     

    }
    txt+="</table>";
    ivt.innerHTML=txt;
   
  });

  
  var quori_mtr_ctrl_sts_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/quori_motor_ctrl_status',
    messageType : 'quori_msgs/MotorControllerStatus'
  });

  function setFieldValue(field,val) {
    var el=document.getElementById(field);
    el.innerHTML=val;
  };

  quori_mtr_ctrl_sts_listener.subscribe(function(message) {
    console.log('Received message on ' + quori_cmd.name + ': ' + toString(message));

    var name = message.controller_name + "_sts";
    mtr_name = name.split('_')[0];

    var mtr=document.getElementById(name);
    var txt = `<table><caption><b>${mtr_name} Motor</b></caption><tr><th>Param</th><th>Value</th><th>Units</th></tr>`;
    
    var tmpu8 = atob(message.temp);
    var temp1 = tmpu8.charCodeAt(0);
    var temp2 = tmpu8.charCodeAt(1);
    //var temp3 = tmpu8.charCodeAt(2);

    if (mtr_name == "Base") {

      setFieldValue("bm_temp1",`${temp1}`);
      setFieldValue("bm_temp2",`${temp2}`);

      setFieldValue("bm_state",`${message.state}`);

      setFieldValue("bm_volts1",`${message.volts[0].toFixed(2)}`);
      setFieldValue("bm_volts2",`${message.volts[1].toFixed(2)}`);
      setFieldValue("bm_volts3",`${message.volts[2].toFixed(2)}`);

      setFieldValue("bm_amps1",`${message.amps[0].toFixed(2)}`);
      setFieldValue("bm_amps2",`${message.amps[1].toFixed(2)}`);

      setFieldValue("bm_di",`0x${message.din.toString(16)}`);

      setFieldValue("bm_fm1",`${message.fm_faults[0]}`);
      setFieldValue("bm_fm2",`${message.fm_faults[1]}`);

      setFieldValue("bm_fs",`${message.fs_flags}`);
      setFieldValue("bm_ff",`${message.ff_faults}`);
      setFieldValue("bm_cec",`${message.coms_error_count}`);

    }
    else {

        setFieldValue(`${mtr_name}_temp`,`${temp1}`);

        setFieldValue(`${mtr_name}_mode`,`${message.state}`);
  
        setFieldValue(`${mtr_name}_volts`,`${message.volts[0].toFixed(2)}`);

        setFieldValue(`${mtr_name}_amps`,`${message.amps[0].toFixed(2)}`);

        setFieldValue(`${mtr_name}_fm1`,`0x${message.fm_faults[0].toString(16)}`);
        setFieldValue(`${mtr_name}_fm2`,`0x${message.fm_faults[1].toString(16)}`);
  
        setFieldValue(`${mtr_name}_fs`,`${message.fs_flags}`);
        setFieldValue(`${mtr_name}_ff`,`${message.ff_faults}`);
        setFieldValue(`${mtr_name}_cec`,`${message.coms_error_count}`);

    }

  });

  var quori_health_sts_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/quori_health_status',
    messageType : 'quori_msgs/QuoriHealthStatus'
  });

	var HF_NONE=0;

	var HF_BASE_BOARD_OK=1;
	var HF_HEAD_BOARD_OK=2;
	var HF_POWER_BOARD_OK=4;
	var HF_BASE_MTR_BOARD_OK=8;

	var HF_ORIN_OK=16;

	var HF_RBTQ_OK=32;

	var HF_TWIST_MOTOR_OK=64;
	var HF_NECK_MOTOR_OK=128;
	var HF_PAN_MOTOR_OK=256;
	var HF_TILT_MOTOR_OK=512;

  function show_health_status_value(tag,flags,bit) {
    var el = document.getElementById(tag);

    if( (flags & bit) == bit) {
      el.innerHTML = "Ok";
      el.style="color: green;";
    }
    else {
      el.innerHTML = "Not Responding";
      el.style="color: red;";
    }
  };


  quori_health_sts_listener.subscribe(function(message) {
      var flags = message.status_flags;
      show_health_status(flags);
  });

  function show_health_status(flags) {
      show_health_status_value("orin_sts",flags,HF_ORIN_OK);
      show_health_status_value("base_board_sts",flags, HF_BASE_BOARD_OK);
      show_health_status_value("head_board_sts",flags, HF_HEAD_BOARD_OK);
      show_health_status_value("power_board_sts",flags, HF_POWER_BOARD_OK);
      show_health_status_value("base_motor_board_sts",flags, HF_BASE_MTR_BOARD_OK);
      show_health_status_value("rbtq_sts",flags, HF_RBTQ_OK);
      show_health_status_value("twist_sts",flags, HF_TWIST_MOTOR_OK);
      show_health_status_value("neck_sts",flags, HF_NECK_MOTOR_OK);
      show_health_status_value("pan_sts",flags, HF_PAN_MOTOR_OK);
      show_health_status_value("tilt_sts",flags, HF_TILT_MOTOR_OK);

  };

  // Publishers
  var quori_cmd = new ROSLIB.Topic({
    ros : ros,
    name : '/quori_cmd',
    messageType : 'quori_msgs/QuoriCmd'
  });

  var quori_cmd_msg = new ROSLIB.Message({
    cmd : 0 ,  // comand
    arg1 : 0,  // first argument
    arg2  :0   // second argument
  });

  function send_quori_cmd(cmd,arg1,arg2) {

    quori_cmd_msg.cmd = cmd;
    quori_cmd_msg.arg1=arg1;
    quori_cmd_msg.arg2=arg2;

    quori_cmd.publish(quori_cmd_msg);
  };

  function showRBTQMtrDetail(field) {

    var el =document.getElementById(field);

    var fm = parseInt(el.innerHTML);
    
    var txt = `<table><caption><b>Base Motor ${field.slice(-1)} Status</b></caption><tr><th>Flag</th><th>Status</th></tr>`;
    var fd=document.getElementById("fault_detail");

    txt+= `<tr><td>Amp Limit</td><td>${fm & 1?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>Stall</td><td>${fm & 2?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>Loop Error</td><td>${fm & 4?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>Quick Stop</td><td>${fm & 8?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>Fwd Limit</td><td>${fm & 16?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>Rev Limit</td><td>${fm & 32?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>Amp trigger</td><td>${fm & 64?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>FETs Off</td><td>${fm & 128?'Set':'Unset'}</td></tr>`;

    txt += "</table>";
    fd.innerHTML = txt;
  };



  function showRBTQFFFaultDetail(field) {
    var el =document.getElementById(field);

    var ff = parseInt(el.innerHTML);
    
    var txt = "<table><caption><b>RBTQ Fault Status</b></caption><tr><th>Flag</th><th>Status</th></tr>";
    var fd=document.getElementById("fault_detail");

    txt+= `<tr><td>Over heat</td><td>${ff & 1?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>Over Volt</td><td>${ff & 2?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>Under Volt</td><td>${ff & 4?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>Short</td><td>${ff & 8?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>E-Stop</td><td>${ff & 16?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>Sensor</td><td>${ff & 32?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>MOS Fail</td><td>${ff & 64?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>Def config</td><td>${ff & 128?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>STO fault</td><td>${ff & 256?'Set':'Unset'}</td></tr>`;

    txt += "</table>";
    fd.innerHTML = txt;
  };

  function showRBTQFlagsDetail(field) {
    var el =document.getElementById(field);

    var fs = parseInt(el.innerHTML);
    
    var txt = "<table><caption><b>RBTQ Flag Status</b></caption><tr><th>Flag</th><th>Status</th></tr>";
    var fd=document.getElementById("fault_detail");

    txt+= `<tr><td>Serial Active</td><td>${fs & 1?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>Pulse Active</td><td>${fs & 2?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>Analog Active</td><td>${fs & 4?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>FETS Off</td><td>${fs & 8?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>Stall</td><td>${fs & 16?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>At limit</td><td>${fs & 32?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>STO</td><td>${fs & 64?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>Script running</td><td>${fs & 128?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td>Running Setup</td><td>${fs & 256?'Set':'Unset'}</td></tr>`;

    txt += "</table>";
    fd.innerHTML = txt;
  };


  function showMoteusDrv1Detail(field) {
    var el =document.getElementById(field);

    var ff = parseInt(el.innerHTML);

    mtr_name = field.split('_')[0];
    
    var txt = `<table><caption><b>Moteus ${mtr_name} DRV STS 1</b></caption><tr><th>Flag</th><th>Status</th></tr>`;
    var fd=document.getElementById("fault_detail");

    txt+= `<tr><td></td><td>${ff & 1?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td></td><td>${ff & 2?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td></td><td>${ff & 4?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td></td><td>${ff & 8?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td></td><td>${ff & 16?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td></td><td>${ff & 32?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td></td><td>${ff & 64?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td></td><td>${ff & 128?'Set':'Unset'}</td></tr>`;

    txt += "</table>";
    fd.innerHTML = txt;
  };

  function showMoteusDrv2Detail(field) {
    var el =document.getElementById(field);

    var ff = parseInt(el.innerHTML);

    mtr_name = field.split('_')[0];
    
    var txt = `<table><caption><b>Moteus ${mtr_name} DRV STS 2</b></caption><tr><th>Flag</th><th>Status</th></tr>`;
    var fd=document.getElementById("fault_detail");

    txt+= `<tr><td></td><td>${ff & 1?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td></td><td>${ff & 2?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td></td><td>${ff & 4?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td></td><td>${ff & 8?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td></td><td>${ff & 16?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td></td><td>${ff & 32?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td></td><td>${ff & 64?'Set':'Unset'}</td></tr>`;
    txt+= `<tr><td></td><td>${ff & 128?'Set':'Unset'}</td></tr>`;

    txt += "</table>";
    fd.innerHTML = txt;
  };

  
  moteus_faults = {
    "0"  : "None",
	"32" : "calibration fault",
	"33" : "motor driver fault" ,
	"34" : "over voltage",
	"35" : "encoder fault",
	"36" : "motor not configured",
	"37" : "pwm cycle overrun",
	"38" : "over temperature",
	"39" : "outside limit" ,
	"40" : "under voltage",
	"41" : "config changed",
	"42" : "theta invalid" ,
	"43" : "position invalid",
	"44" : "driver enable fault",
	"45" : "stop position deprecated" ,
	"46" : "timing violation" ,
	"47" : "bemf feedforward no accel"
};

  function showMoteusFaultCodeDetail(field) {
    var el =document.getElementById(field);

    var ff = parseInt(el.innerHTML);

    mtr_name = field.split('_')[0];
    
    var txt = `<table><caption><b>Moteus ${mtr_name} Fault Code</b></caption><tr><th>Flag</th><th>Status</th></tr>`;
    var fd=document.getElementById("fault_detail");

    txt+= `<tr><td>Fault reason</td><td>${moteus_faults[ff]}</td></tr>`;


    txt += "</table>";
    fd.innerHTML = txt;
  };


  function showMoteusFlagsDetail(field) {
    var el =document.getElementById(field);

    var ff = parseInt(el.innerHTML);

    mtr_name = field.split('_')[0];
    
    var txt = `<table><caption><b>Moteus ${mtr_name} Flags</b></caption><tr><th>Flag</th><th>Status</th></tr>`;
    var fd=document.getElementById("fault_detail");

    var homed= ff & 0xFF00;
    var move_done = ff & 0xFF;

    txt+= `<tr><td>Home State</td><td>${homed}</td></tr>`;
    txt+= `<tr><td>Move Done</td><td>${move_done!=0?'Set':'Unset'}</td></tr>`;
   

    txt += "</table>";
    fd.innerHTML = txt;
  };
