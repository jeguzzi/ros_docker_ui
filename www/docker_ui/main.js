// Connecting to ROS
// -----------------

ns = '';
hostname = window.location.hostname;


var camera_url = "http://"+hostname+":8080/stream?topic="+ns+"/camera/image_raw&quality=50";
$("#camera").attr('src', camera_url);

var ros = new ROSLIB.Ros({
    url: 'ws://' + hostname + ':9090'
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});


function call(srv, name) {
    console.log('call ' + srv + ' ' + name)
    var request = new ROSLIB.ServiceRequest({
        name: name,
    });
    srv.callService(request, function(result) {
        console.log(result.response);
    });
}

function stop(name) {
    call(stop_srv, name)
}

function start(name) {
    call(start_srv, name)
}

function pause(name) {
    call(pause_srv, name)
}

function unpause(name) {
    call(unpause_srv, name)
}

var levels = {}
levels[0] = {
    "class": "success",
    "name": "OK",
    "glyph": "ok-sign"
}
levels[1] = {
    "class": "warning",
    "name": "WARN",
    "glyph": "exclamation-sign"
}
levels[2] = {
    "class": "danger",
    "name": "ERROR",
    "glyph": "minus-sign"

}
levels[3] = {
    "class": "danger",
    "name": "STALE",
    "glyph": "arrow-down"
}

_state2class = {
    "running": "success",
    "exited": "danger",
    "paused": "warning",
    "removed": "info"
}


String.prototype.format = function() {
    var formatted = this;
    for (var arg in arguments) {
        formatted = formatted.replace("{" + arg + "}", arguments[arg]);
    }
    return formatted;
};


var _alerts = {};
var num_alerts = [0, 0, 0, 0]
var _containers = {};

function add_to_logs(m) {
    level = levels[m.level]
    $("#log").append(
        "<tr class='{0}'><td>{1}</td><td>{2}</td><td>{3}</td></tr>".format(level.class, ds, m.name, m.message));
}

function add_to_alerts(m) {
    var i = m.name.replace(/[\s//:]+/g, '');
    var level = levels[m.level]
    var p = $("#_a" + m.level)
    var nl = m.level;
    if (!(i in _alerts)) {
        _alerts[i] = m.level;
        num_alerts[nl]++;
        n = $(`
<div id='{0}' class="panel">
<div class="panel-heading">
<h4 class="panel-title">

</h4>
</div>
<div id="t{2}" class="panel-collapse collapse">
</div>
</div>
`.format(i, i, i))

        p.append(n)
        $("#_n" + nl).text(num_alerts[nl]);
    } else {
        n = $("#" + i)
        var ol = _alerts[i];
        if (ol != nl) {
            num_alerts[ol]--;
            num_alerts[nl]++;
            $("_n" + nl).text(num_alerts[nl]);
            $("_n" + ol).text(num_alerts[ol]);
            n.detach()
            p.append(n)
        }
    }
    var h = n.find('.panel-title')

    var etc;
    if (m.values.length) {
        etc = "...";
    } else {
        etc = "";
    }

    var title = $("<span class='glyphicon glyphicon-{0}'></span><span><em>{1}</em>: {2} {3}</span>".format(level.glyph, m.name, m.message, etc))

    var a;
    if (m.values.length) {
        a = $("<a data-toggle='collapse' href='#t" + i + "'></a>")
        h.html(a)
    } else {
        a = h
    }
    a.html(title)


    var s = "<ul class='list-group'>"
    for (var i in m.values) {
        item = m.values[i];
        value = item.value;
        key = item.key;
        s += "<li class='list-group-item'>" + key + ": " + value + "</li>"
    }
    s += "</ul>"
    n.find('.panel-collapse').html(s)
        //n.attr("data-content", s)
    n.attr("class", "panel panel-{0}".format(level.class));
}

function add_log(date, level, name, msg)
{
   console.log(date+level+name+msg);
   cls = {
    'INFO':'info',
    'DEBUG':'info',
    'WARN':'warning',
    'ERROR': 'danger',
    'FATAL':'danger'}[level]
   $("#logs").append(
     `
     <tr class={4}>
     <td>
     {0}
     </td>
     <td>
     {1}
     </td>
     <td>
     {2}
     </td>
     <td>
     {3}
     </td>
     </tr>
     `
     .format(date, level, name, msg,cls)
  )
}

function add_to_containers(m) {
    var i = m.name.replace(/[\s//:]+/g, '');
    var level = levels[m.level]
    if (!(i in _containers)) {
        _containers[i] = m;
        $("#containers").append(
            `<tr id='_C{0}'>
           <td style="vertical-align:middle; text-align:center"><strong>{1}</strong></td>
           <td>
           <button type="button" class="btn btn-success" onclick="start('{2}')">Start</button>
           </td>
           <td>
           <button type="button" class="btn btn-warning" onclick="pause('{3}')">Pause</button>
           </td>
           <td>
           <button type="button" class="btn btn-danger" onclick="stop('{4}')">Stop</button>
           </td>
           <td>
           </td>
           <td>
           </td>
           <td>
             <input type="checkbox" name="my-checkbox" disabled>
           </td>
           </tr>
           `
            .format(i, i, i, i, i))
    }
    //console.log(m.name+ " "+m.cpu)
    var n = $("#_C" + i)
    var cc = n.children()
    var c1 = $(cc[0])
    c1.attr("class", _state2class[m.status])
    var bs = [1, 2, 3].map(function(j) {
        return $(cc[j]).find('button');
    });
    switch (m.status) {
        case 'running':
            //$(bs[0]).prop('disabled', true)
            $(bs[0]).hide()
                //$(bs[1]).prop('disabled', false)
            $(bs[1]).text('Pause')
            $(bs[1]).attr("onclick", "pause('{0}')".format(i))
            $(bs[1]).show()
            $(bs[2]).show()
                //$(bs[2]).prop('disabled', false)
            break;
        case 'paused':
            //$(bs[0]).prop('disabled', true)
            $(bs[0]).hide()
                //$(bs[1]).prop('disabled', false)
            $(bs[1]).text('Unpause')
            $(bs[1]).attr("onclick", "unpause('{0}')".format(i))
            $(bs[1]).show()
            $(bs[2]).hide()
                //$(bs[2]).prop('disabled', true)
            break;
        case 'exited':
            //$(bs[0]).prop('disabled', false)
            $(bs[0]).show()
                //$(bs[1]).prop('disabled', true)
            $(bs[1]).hide()
                //$(bs[2]).prop('disabled', true)
            $(bs[2]).hide()
            break;
        default:
    }

    //...
    var c3 = $(cc[4])
    c3.text('{0}%'.format((100 * m.cpu).toFixed(1)))
    var c4 = $(cc[5])
    if (m.memory_limit > 0) {
        mem_p = (100 * m.memory_usage / m.memory_limit).toFixed(1)
    } else {
        mem_p = 0
    }
    c4.text('{0} MB ({1}%)'.format((m.memory_usage).toFixed(0), mem_p))
    var c5 = $(cc[6])
    var s = c5.find('input')
    if (m.restart_policy == 'always') {
        s.prop('checked', true);
    } else {
        s.prop('checked', false);
    }
    //s.prop('enabled',true);
    s.bootstrapSwitch('disabled',false);
    //Stil need to implement on the server
    s.on('switchChange.bootstrapSwitch', function(event, state) {
        set_restart(i, state)
    });
}


function set_restart(i, state) {
    var policy = state ? 'always' : 'no'
    console.log('set restart ' + i + ' to ' + policy)
    var request = new ROSLIB.ServiceRequest({
        name: i,
        policy: policy,
        max_retry: 0
    });
    restart_policy_srv.callService(request, function(result) {
        console.log(result.response);
    });
}


var empty = new ROSLIB.Message({})

var listener = new ROSLIB.Topic({
    ros: ros,
    name: '/diagnostics',
    messageType: 'diagnostic_msgs/DiagnosticArray'
});
listener.subscribe(function(ms) {
    stamp = ms.header.stamp
    date = new Date(stamp.secs * 1000 + stamp.nsecs / 1000000)
    ds = date.toLocaleTimeString("en-CH")
    for (var i = 0; i < ms.status.length; i++) {
        m = ms.status[i];
        add_to_alerts(m);
    }
});

var listener = new ROSLIB.Topic({
    ros: ros,
    name: '/rosout_agg',
    messageType: 'rosgraph_msgs/Log'
});
listener.subscribe(function(ms) {
    var stamp = ms.header.stamp
    var date = new Date(stamp.secs * 1000 + stamp.nsecs / 1000000)
    var ds = date.toLocaleTimeString("en-CH")
    var level = {1:'DEBUG', 2:'INFO', 4:'WARN', 8:'ERROR', 16:'FATAL'}[ms.level]
    var name = ms.name
    var msg = ms.msg
    add_log(ds, level, name, msg)
});


var d_listener = new ROSLIB.Topic({
    ros: ros,
    name: ns+'/docker/containers',
    messageType: 'docker_ui/DockerContainers'
});
d_listener.subscribe(function(ms) {
    //console.log(ms);
    var stamp = ms.header.stamp
    var date = new Date(stamp.secs * 1000 + stamp.nsecs / 1000000)
    var ds = date.toLocaleTimeString("en-CH")
    for (var i = 0; i < ms.containers.length; i++) {
        var m = ms.containers[i];
        add_to_containers(m);
    }
});


stop_srv = new ROSLIB.Service({
    ros: ros,
    name: ns+'/docker/stop',
    serviceType: 'docker_ui/DockerContainerCmd'
});

start_srv = new ROSLIB.Service({
    ros: ros,
    name: ns+'/docker/start',
    serviceType: 'docker_ui/DockerContainerCmd'
});

pause_srv = new ROSLIB.Service({
    ros: ros,
    name: ns+'/docker/pause',
    serviceType: 'docker_ui/DockerContainerCmd'
});

unpause_srv = new ROSLIB.Service({
    ros: ros,
    name: ns+'/docker/unpause',
    serviceType: 'docker_ui/DockerContainerCmd'
});
restart_policy_srv = new ROSLIB.Service({
    ros: ros,
    name: ns+'/docker/restart_policy',
    serviceType: 'docker_ui/DockerSetRestartContainer'
});
