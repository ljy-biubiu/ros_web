var path = null;
var filename = null;
import * as THREE from './three.js/build/three.module.js';
import { PCDLoader } from './three.js/examples/jsm/loaders/PCDLoader.js';
import { TrackballControls } from './three.js/examples/jsm/controls/TrackballControls.js';

var scene = new THREE.Scene();
scene.background = new THREE.Color(0x000000);

var camera = new THREE.PerspectiveCamera(15, window.innerWidth / window.innerHeight, 0.01, 40);     //透视摄像机 这个camera拟似现实 用它就对了
camera.position.x = 0.4;
camera.position.z = 15;                                                                            //position 表示对象局部位置的Vector3。默认值为(0, 0, 0)。
camera.up.set(0, 0, 1);                                                                             //这个属性由lookAt方法所使用，例如，来决定结果的朝向。 默认值是Object3D.DefaultUp，即( 0, 1, 0 )。                              
console.log("-----------------------------------------------");
scene.add(camera);


var renderer = new THREE.WebGLRenderer({ antialias: true });                                        //WebGLRenderer渲染器      antialias抗 锯齿                             
renderer.setPixelRatio(window.devicePixelRatio);                                                    //设置像素比
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);                                                     //添加到<body>的内容

var controls = new TrackballControls(camera, renderer.domElement);                                  //轨迹球控件，最常用的控件，可以使用鼠标轻松的移动、平移，缩放场景。

controls.rotateSpeed = 2.0;                                                                         //鼠标控制速度
controls.zoomSpeed = 0.3;                                                                           //鼠标控制速度
controls.panSpeed = 0.2;                                                                            //鼠标控制速度

controls.staticMoving = true;                                                                       // 静止移动，为 true 则没有惯性

controls.minDistance = 0.3; 
controls.maxDistance = 0.3 * 100;

var loader = new PCDLoader();                                                                       //创建pcd加载对象

function clear_scene() {
    scene.remove.apply(scene, scene.children);                                                      //清除场景中的所有对象
}
window.clear_scene = clear_scene;                                                                   //？？？？？？？？？？？？？

function load() {
    filename = document.getElementById("filename").value;                                           //读取文件地址
    path = "pcd/" + filename;
    loader.load(path, function (points) {

    clear_scene();
    points.scale.multiplyScalar(0.01);                                                         //物体的局部缩放。默认值是Vector3( 1, 1, 1 )。 
    points.geometry.center();
    scene.add(points);
    console.log(points);
    var center = points.geometry.boundingSphere.center;
    controls.target.set(center.x, center.y, center.z);
    controls.update();
    var geometry11 = new THREE.BoxGeometry(100, 100, 100);
    var material11 = new THREE.MeshBasicMaterial({ color: 0xffffff });
    var cube = new THREE.Mesh(geometry11, material11);
    cube.position.x=0;
    cube.position.y=0;
    cube.position.z=0;
    

        console.log("-----------------------------------------------");
        scene.add(cube);

        animate();                                                                                //更新动画


    },
        null,
        function (error) {
            alert(error.target.statusText); 1
        });
}
window.load = load;                                                                                //？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？

var container = document.createElement('div');                                                     //直接采用代码建立一个DIV
document.body.appendChild(container);                                                              //DIV放进body里面
container.appendChild(renderer.domElement);                                                        //domElement画布

window.addEventListener('resize', onWindowResize, false);                                          //根据浏览器窗口的变动更新相机的aspect以及 渲染器的渲染范围，实现自适应的效果。
window.addEventListener('keypress', keyboard);

var animate = function () {
    requestAnimationFrame(animate);
    controls.update();
    // console.log("-------------------------");
    // cube.rotation.x += 0.01;
    // cube.rotation.y += 0.01;
    renderer.render(scene, camera);
};

function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;                                        //摄像机视锥体的长宽比，通常是使用画布的宽/画布的高。默认值是1（正方形画布）。
    camera.updateProjectionMatrix();                                                               //更新摄像机投影矩阵。在任何参数被改变以后必须被调用。
    renderer.setSize(window.innerWidth, window.innerHeight);
    controls.handleResize();
}

function keyboard(ev) {                                                                            //键盘事件
    if (filename == null) return;

    var points = scene.getObjectByName(filename);

    switch (ev.key || String.fromCharCode(ev.keyCode || ev.charCode)) {

        case '+':
            points.material.size *= 1.2;
            points.material.needsUpdate = true;
            break;

        case '-':
            points.material.size /= 1.2;
            points.material.needsUpdate = true;
            break;
    }

}

window.onload = function() {
    var filename = window.location.hash.substr(1).split('&'); // substr(1) to remove the `#`
    if (filename != "") {
        document.getElementById("filename").value = filename;
        load();
    }
}


window.onhashchange = window.onload;







