// Copyright 2009 FriendFeed
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may
// not use this file except in compliance with the License. You may obtain
// a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
// License for the specific language governing permissions and limitations
// under the License.

$(document).ready(function() {
    if (!window.console) window.console = {};
    if (!window.console.log) window.console.log = function() {};

    sol_updater.start();
});

var sol_updater = {
    socket: null,

    start: function() {
        var url = "ws://" + location.host + "/rtkroversolutionsocket";
        sol_updater.socket = new WebSocket(url);
        sol_updater.socket.onmessage = function(event) {
            sol_updater.showMessage(JSON.parse(event.data));
        }
    },

    showMessage: function(message) {
	// these are the fields in the message
	// field_names = ['date', 'time_gps_utc', 'time_rcv_utc', 'e_baseline_m', 'n_baseline_m',
	//                'u_baseline_m', 'status', 'n_satelites', 'std_e_m', 'std_n_m',
	//                'std_u_m', 'sde_en_m', 'sde_nu_m', 'sde_ue_m', 'age', 'ar_ratio',
	//                'velocity_e_ms', 'velocity_n_ms', 'velocity_u_ms', 'std_ve',
	//                'std_vn', 'std_vu', 'std_ven', 'std_vnu', 'std_vue']

	// 'fix','float', 'single' - unlikely to be 'sbas', 'dgps', 'ppp'
        $("#rtk_status").text(message.status);

	$("#rtk_baseline").text('e:' + message.e_baseline_m + ' n:' + message.n_baseline_m + ' u:'+ message.u_baseline_m);
	$("#rtk_baseline_std").text('std e:' + message.std_e_m + ' std n:' + message.std_n_m + ' std u:'+ message.std_u_m);
    }
};
