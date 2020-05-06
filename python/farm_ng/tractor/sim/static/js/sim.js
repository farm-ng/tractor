import * as THREE from '/third_party/three.module.js';

import { OrbitControls } from '/third_party/OrbitControls.js';
import { STLLoader } from '/third_party/STLLoader.js';

var camera, controls, scene, renderer, tractor_mesh, x_axis;


var sim_updater = {
    socket: null,

    start: function() {
        var url = "ws://" + location.host + "/simsocket";
        sim_updater.socket = new WebSocket(url);
        sim_updater.socket.onmessage = function(event) {
            sim_updater.showMessage(JSON.parse(event.data));
        }
    },

    showMessage: function(message) {
	var p = message.world_translation_tractor;
	var q = message.world_quaternion_tractor;
	tractor_mesh.position.set(p[0],p[1],p[2]);
	tractor_mesh.quaternion.set(q[0],q[1],q[2],q[3]);
	x_axis.position.set(p[0],p[1],p[2]);
	x_axis.quaternion.set(q[0],q[1],q[2],q[3]);

    }
};

init();
//render(); // remove when using next line for animation loop (requestAnimationFrame)
animate();

function init() {

    scene = new THREE.Scene();
    scene.background = new THREE.Color( 0xcccccc );
    scene.fog = new THREE.FogExp2( 0xcccccc, 0.02 );

    renderer = new THREE.WebGLRenderer( { antialias: true } );
    renderer.setPixelRatio( window.devicePixelRatio );
    renderer.setSize( window.innerWidth, window.innerHeight );
    document.body.appendChild( renderer.domElement );

    camera = new THREE.PerspectiveCamera( 60, window.innerWidth / window.innerHeight, 0.001, 500 );
    camera.up.set(0,0,1);
    camera.position.set(2.5, 2.5, 2.5);
    camera.updateMatrix();


    camera.updateMatrix();

    var size = 400;
    var divisions = 400;

    var grid = new THREE.GridHelper( size, divisions );
    grid.geometry.rotateX(- Math.PI / 2 );
    scene.add( grid);


    // controls

    controls = new OrbitControls( camera, renderer.domElement );

    controls.enableDamping = true; // an animation loop is required when either damping or auto-rotation are enabled
    controls.dampingFactor = 0.05;

    controls.screenSpacePanning = false;

    controls.minDistance = 0.1;
    controls.maxDistance = 100;

    controls.maxPolarAngle = Math.PI / 2;

    // world


    var loader = new STLLoader();
    loader.load( './static/tractor.v0.stl', function ( geometry ) {

	var material = new THREE.MeshPhongMaterial( { color: 0xff5533, specular: 0x111111, shininess: 200 } );
	tractor_mesh = new THREE.Mesh( geometry, material );

	tractor_mesh.position.set( 0, 0, 0 );
	tractor_mesh.rotation.set( 0, 0, 0 );
	tractor_mesh.scale.set( 0.001, 0.001, 0.001);

	tractor_mesh.castShadow = true;
	tractor_mesh.receiveShadow = true;
	scene.add( tractor_mesh);

	var length = 1;
	x_axis = new THREE.AxesHelper()
	//var x_axis = new THREE.ArrowHelper( THREE.Vector(1,0,0), THREE.Vector3( 0, 0, 0 ), length, 0xff0000 );
	scene.add(x_axis);
	//var y_axis = new THREE.ArrowHelper( new THREE.Vector(0,1,0), new THREE.Vector3( 0, 0, 0 ), length, 0x00ff00 );
	//scene.add(y_axis);
	//var z_axis = new THREE.ArrowHelper( new THREE.Vector(0,0,1), new THREE.Vector3( 0, 0, 0 ), length, 0x0000ff );
	//scene.add(z_axis);

    } );


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

    sim_updater.start();
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
