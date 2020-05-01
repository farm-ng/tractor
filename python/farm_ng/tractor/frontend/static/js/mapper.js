import * as THREE from './third_party/three.module.js';

import { OrbitControls } from './third_party/OrbitControls.js';

var camera, controls, scene, renderer, meshes, gps_grid;


var mapper_sol_updater = {
    socket: null,

    start: function() {
        var url = "ws://" + location.host + "/rtkroversolutionsocket";
        mapper_sol_updater.socket = new WebSocket(url);
        mapper_sol_updater.socket.onmessage = function(event) {
            mapper_sol_updater.showMessage(JSON.parse(event.data));
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
        //$("#rtk_status").text(message.status);

	//$("#rtk_baseline").text('e:' + message.e_baseline_m + ' n:' + message.n_baseline_m + ' u:'+ message.u_baseline_m);
	//$("#rtk_baseline_std").text('std e:' + message.std_e_m + ' std n:' + message.std_n_m + ' std u:'+ message.std_u_m);

	var pos_enu = new THREE.Vector3(message.e_baseline_m, message.n_baseline_m, message.u_baseline_m);
	var std_enu = new THREE.Vector3(message.std_e_m, message.std_n_m,message.std_u_m);

	var fix_color = 0xffffff;
	if( message.status == "fix") {
	    fix_color = 0x00ff00;
	} else if (message.status == "float") {
	    fix_color = 0xffff00;
	} else if (message.status == "single") {
	    fix_color = 0xff0000;
	}
	var geometry = new THREE.SphereBufferGeometry(1);
	var material = new THREE.MeshPhongMaterial( { color: fix_color, flatShading: true } );

	var mesh = new THREE.Mesh( geometry, material );
	mesh.position.copy(pos_enu);
	mesh.scale.copy(std_enu.multiplyScalar(3.0));
	mesh.updateMatrix();
	mesh.matrixAutoUpdate = false;
	meshes.push(mesh)
	scene.add( mesh );
	gps_grid.position.set(pos_enu.x, pos_enu.y, pos_enu.z-1.0);
	controls.target.copy(pos_enu);
	if (meshes.length > 100000) {
	    scene.remove(meshes.shift());
	}
    }
};

init();
//render(); // remove when using next line for animation loop (requestAnimationFrame)
animate();

function init() {
    meshes = new Array();

    scene = new THREE.Scene();
    scene.background = new THREE.Color( 0xcccccc );
    //scene.fog = new THREE.FogExp2( 0xcccccc, 0.02 );

    renderer = new THREE.WebGLRenderer( { antialias: true } );
    renderer.setPixelRatio( window.devicePixelRatio );
    renderer.setSize( window.innerWidth, window.innerHeight );
    document.body.appendChild( renderer.domElement );

    camera = new THREE.PerspectiveCamera( 60, window.innerWidth / window.innerHeight, 0.001, 500 );
    camera.position.set(20, -40, 5);
    camera.up.set(0,0,1);


    camera.updateMatrix();

    var size = 400;
    var divisions = 40;

    var gridHelper = new THREE.GridHelper( size, divisions );
    gridHelper.geometry.rotateX(- Math.PI / 2 );
    scene.add( gridHelper);

    gps_grid = new THREE.GridHelper( 1.0, 10);
    gps_grid.geometry.rotateX(- Math.PI / 2 );
    gps_grid.updateMatrix();
    gps_grid.matrixAutoUpdate = true;

    scene.add( gps_grid );


    // controls

    controls = new OrbitControls( camera, renderer.domElement );
    controls.target.set(22.7326,-33.2568,-4.6393);
    //controls.addEventListener( 'change', render ); // call this only in static scenes (i.e., if there is no animation loop)

    controls.enableDamping = true; // an animation loop is required when either damping or auto-rotation are enabled
    controls.dampingFactor = 0.05;

    controls.screenSpacePanning = false;

    controls.minDistance = 0.1;
    controls.maxDistance = 100;

    controls.maxPolarAngle = Math.PI / 2;

    // world



    // lights

    var light = new THREE.DirectionalLight( 0xffffff );
    light.position.set( 1, 1, 1 );
    scene.add( light );

    var light = new THREE.DirectionalLight( 0x002288 );
    light.position.set( - 1, - 1, - 1 );
    scene.add( light );

    var light = new THREE.AmbientLight( 0x222222 );
    scene.add( light );

    //

    window.addEventListener( 'resize', onWindowResize, false );

    mapper_sol_updater.start();
}

function onWindowResize() {

    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();

    renderer.setSize( window.innerWidth, window.innerHeight );

}

function animate() {

    requestAnimationFrame( animate );

    controls.update(); // only required if controls.enableDamping = true, or if controls.autoRotate = true

    render();

}

function render() {

    renderer.render( scene, camera );

}
