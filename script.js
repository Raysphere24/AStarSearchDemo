"use strict";

let model;
let lineSegmentBuffer;
let lineBuffer;
let pointsBuffer;
let queue;
let intervalID;
let isShortestPathTreeSearch, isAStarSearch;
let start, goal;
let isSearching = false;

// Class to represent a vertex of model
class Vertex {
	constructor(position) {
		this.position = position;
		this.neighbors = [];
		this.isClosed = false;
		this.previous = null;
	}
}

// Distance between vertices a and b
function distance(a, b) { return a.position.distanceTo(b.position); }

// Class to represent search node
class SearchNode {
	constructor(parent, target, f, g) {
		this.parent = parent;
		this.target = target;
		this.f = f;
		this.g = g;
	}
}

function compareNodeFunction(a, b) { return a.f - b.f; }

// Class to render model and search
class Model {
	/** @param {string} text the content of .obj file */
	constructor(text) {
		this.vertices = [];

		const vertexArray = [];
		const edgeArray = [];
		const faceArray = [];

		const lines = text.split("\n");

		for (const line of lines) {
			const s = line.split(" ");
			if (s[0] === "v") {
				const x = parseFloat(s[1]) + 0.02;
				const y = parseFloat(s[2]) - 0.1;
				const z = parseFloat(s[3]);

				vertexArray.push(x, y, z);

				this.vertices.push(new Vertex(new THREE.Vector3(x, y, z)));
			} else if (s[0] === "f") {
				const i = parseInt(s[1]) - 1;
				const j = parseInt(s[2]) - 1;
				const k = parseInt(s[3]) - 1;

				edgeArray.push(i, j, j, k, k, i);
				faceArray.push(i, j, k);

				this.vertices[i].neighbors.push(this.vertices[j]);
				this.vertices[j].neighbors.push(this.vertices[k]);
				this.vertices[k].neighbors.push(this.vertices[i]);
			}
		}

		const attribute = new THREE.Float32BufferAttribute(vertexArray, 3);

		const meshGeometry = new THREE.BufferGeometry();
		meshGeometry.addAttribute("position", attribute);
		meshGeometry.setIndex(new THREE.Uint16BufferAttribute(faceArray, 1));

		this.mesh = new THREE.Mesh(meshGeometry, new THREE.MeshBasicMaterial({
			color: 0x000000,
			polygonOffset: true,
			polygonOffsetFactor: 1,
			polygonOffsetUnits: 1,
		}));
		scene.add(this.mesh);

		const lineGeometry = new THREE.BufferGeometry();
		lineGeometry.addAttribute("position", attribute);
		lineGeometry.setIndex(new THREE.Uint16BufferAttribute(edgeArray, 1));

		this.backLines = new THREE.LineSegments(lineGeometry, new THREE.LineBasicMaterial({
			color: 0x404040,
			depthFunc: THREE.GreaterDepth,
			depthWrite: false,
		}));
		scene.add(this.backLines);

		this.frontLines = new THREE.LineSegments(lineGeometry, new THREE.LineBasicMaterial({
			color: 0x808080,
		}));
		scene.add(this.frontLines);
	}

	/** @param {Vertex} start */
	searchInitialize(start) {
		for (const vertex of this.vertices) {
			vertex.isClosed = false;
		}

		queue = new TinyQueue(compareNodeFunction);
		queue.push(new SearchNode(null, start, 0, 0));

		lineSegmentBuffer.clear();
		lineBuffer.clear();
	}

	searchStepTree() {
		const node = queue.pop();

		if (node === undefined) return true;

		if (node.target.isClosed === true) return false;

		node.target.isClosed = true;
		node.target.previous = node.parent;

		if (node.parent !== null) {
			lineSegmentBuffer.push(node.parent.position);
			lineSegmentBuffer.push(node.target.position);
		}

		for (const neighbor of node.target.neighbors) {
			if (neighbor.isClosed === true) continue;

			const f = node.f + distance(node.target, neighbor);
			queue.push(new SearchNode(node.target, neighbor, f, 0));
		}

		return false;
	}

	searchStep() {
		const node = queue.pop();

		if (node === undefined) return true;

		if (node.target.isClosed === true) return false;

		node.target.isClosed = true;
		node.target.previous = node.parent;

		if (node.target === goal) {
			// Reconstruct Path
			let vertex = goal;
			while (vertex) {
				lineBuffer.push(vertex.position);
				vertex = vertex.previous;
			}
			return true;
		}

		for (const neighbor of node.target.neighbors) {
			if (neighbor.isClosed) continue;

			const g = node.g + distance(node.target, neighbor);
			const h = isAStarSearch ? distance(neighbor, goal) : 0;
			queue.push(new SearchNode(node.target, neighbor, g + h, g));

			lineSegmentBuffer.push(node.target.position);
			lineSegmentBuffer.push(neighbor.position);
		}
	}

	/** @param {THREE.Vector3} */
	findNearestVertex(pos) {
		let minDistSqr = Infinity;
		let ans = null;

		for (const vertex of this.vertices) {
			const distSqr = pos.distanceToSquared(vertex.position);
			if (distSqr < minDistSqr) {
				minDistSqr = distSqr;
				ans = vertex;
			}
		}

		return ans;
	}
}

// Class to render lines
class LineBuffer {
	/**
	 * @param {function} type THREE.Line or THREE.LineSegments constructor
	 * @param {number} capacity maximum of the number of line segments
	 */
	constructor(type, capacity, backColor, frontColor) {
		this.count = 0;
		this.capacity = capacity;

		this.vertices = new Float32Array(this.capacity * 3);
		this.geometry = new THREE.BufferGeometry();

		this.geometry.addAttribute("position", new THREE.BufferAttribute(this.vertices, 3));
		this.geometry.setDrawRange(0, 0);

		this.back = new type(this.geometry, new THREE.LineBasicMaterial({
			color: backColor,
			depthFunc: THREE.GreaterDepth,
			depthWrite: false,
		}));

		this.front = new type(this.geometry, new THREE.LineBasicMaterial({
			color: frontColor,
		}));
	}

	push(pos) {
		const n = this.count++;

		this.vertices[3*n+0] = pos.x;
		this.vertices[3*n+1] = pos.y;
		this.vertices[3*n+2] = pos.z;

		this.geometry.setDrawRange(0, n+1);
		this.geometry.attributes.position.needsUpdate = true;
	}

	clear() {
		this.count = 0;
		this.geometry.setDrawRange(0, 0);
	}
}

// Class to render start/goal points
class PointsBuffer {
	constructor() {
		this.vertices = new Float32Array(6);
		this.geometry = new THREE.BufferGeometry();

		this.geometry.addAttribute("position", new THREE.BufferAttribute(this.vertices, 3));
		this.geometry.setDrawRange(0, 0);

		this.points = new THREE.Points(this.geometry, new THREE.PointsMaterial({
			color: 0xff0000,
			size: 1 / 128,
			depthTest: false,
		}));
	}

	setStart(pos) {
		this.vertices[0] = pos.x;
		this.vertices[1] = pos.y;
		this.vertices[2] = pos.z;

		this.geometry.setDrawRange(0, 1);
		this.geometry.attributes.position.needsUpdate = true;
	}

	setGoal(pos) {
		this.vertices[3] = pos.x;
		this.vertices[4] = pos.y;
		this.vertices[5] = pos.z;

		this.geometry.setDrawRange(0, 2);
		this.geometry.attributes.position.needsUpdate = true;
	}
}

function render() {
	renderer.render(scene, camera);
};

function update() {
	window.requestAnimationFrame(render);
}

// Called periodically when in animation
function intervalFunc() {
	for (let i = 0; i < 32; i++) {
		let finished;
		if (isShortestPathTreeSearch)
			finished = model.searchStepTree();
		else
			finished = model.searchStep();

		if (finished) {
			clearInterval(intervalID);
			isSearching = false;
			break;
		}
	}
	render();
}

/** @type {HTMLCanvasElement} */
const canvas = document.getElementById("canvas");

const isAStarSearchRadio = document.getElementById("isAStarSearchRadio");
const isShortestPathTreeRadio = document.getElementById("isShortestPathTreeRadio");
const animationCheckbox = document.getElementById("animationCheckbox");
const showHiddenLinesCheckbox = document.getElementById("showHiddenLinesCheckbox");

isShortestPathTreeRadio.addEventListener("change", ev => { start = undefined; });

showHiddenLinesCheckbox.addEventListener("change", ev => {
	model.backLines.visible = showHiddenLinesCheckbox.checked;
	lineSegmentBuffer.back.visible = showHiddenLinesCheckbox.checked;
	lineBuffer.back.visible = showHiddenLinesCheckbox.checked;
	update();
});

const renderer = new THREE.WebGLRenderer({
	canvas: canvas,
	antialias: true,
	stencil: false,
});

const scene = new THREE.Scene();
scene.rotation.x = 0.5;

// Canvas is square, so aspect is 1
const camera = new THREE.PerspectiveCamera(24, 1, 0.25, 1);
camera.position.z = 0.5;

function resize() {
	const width = canvas.offsetWidth;
	const height = canvas.offsetHeight;

	// Canvas is square, so height check is omitted
	if (width * window.devicePixelRatio === canvas.width) return;

	renderer.setPixelRatio(window.devicePixelRatio);
	renderer.setSize(width, height, false);

	update();
}
resize();
window.addEventListener("resize", resize);

new THREE.FileLoader().load("Bunny.obj", text => {
	model = new Model(text);

	lineSegmentBuffer = new LineBuffer(THREE.LineSegments, 1 << 14, 0x008000, 0x00ff00);
	scene.add(lineSegmentBuffer.back);
	scene.add(lineSegmentBuffer.front);

	lineBuffer = new LineBuffer(THREE.Line, 1 << 10, 0x800000, 0xff0000);
	scene.add(lineBuffer.back);
	scene.add(lineBuffer.front);

	pointsBuffer = new PointsBuffer();
	scene.add(pointsBuffer.points);

	update();
});

let mouseDownX = 0, mouseDownY = 0;
let hasMouseMoved = false;

canvas.addEventListener("mousedown", ev => {
	// ignore other than left button is being pressed
	if (ev.button !== 0) return;

	mouseDownX = ev.x;
	mouseDownY = ev.y;
	hasMouseMoved = false;
});

canvas.addEventListener("mousemove", ev => {
	// ignore other than only left button is pressed
	if (ev.buttons !== 1) return;

	const delta = Math.abs(ev.x - mouseDownX) + Math.abs(ev.y - mouseDownY);
	if (hasMouseMoved || delta >= 4) {
		scene.rotation.x += ev.movementY * 8 / window.innerHeight;
		scene.rotation.y += ev.movementX * 8 / window.innerHeight;
		hasMouseMoved = true;

		update();
	}
});

canvas.addEventListener("mouseup", ev => {
	// ignore other than left button is being released
	if (ev.button !== 0) return;

	if (hasMouseMoved || !model) return;

	const raycaster = new THREE.Raycaster();
	const coords = new THREE.Vector2(
		ev.offsetX / canvas.offsetWidth * 2 - 1,
		ev.offsetY / canvas.offsetHeight * -2 + 1
	);
	raycaster.setFromCamera(coords, camera);
	const result = raycaster.intersectObject(model.mesh);
	if (result.length === 0) return; // when mouse is in background

	const pos = scene.worldToLocal(result[0].point);
	const nearest = model.findNearestVertex(pos);

	isShortestPathTreeSearch = isShortestPathTreeRadio.checked;
	isAStarSearch = isAStarSearchRadio.checked;
	let isReady;

	if (isShortestPathTreeSearch) {
		start = nearest;
		pointsBuffer.setStart(start.position);
		isReady = true;
	} else {
		if (!start || goal) {
			start = nearest;
			goal = undefined;
			pointsBuffer.setStart(start.position);
			isReady = false;
		} else {
			goal = nearest;
			pointsBuffer.setGoal(goal.position);
			isReady = true;
		}
	}

	if (isReady) {
		model.searchInitialize(start);
		if (animationCheckbox.checked) {
			if (!isSearching) {
				intervalID = window.setInterval(intervalFunc, 16);
				isSearching = true;
			}
		} else {
			if (isShortestPathTreeSearch) {
				while (!model.searchStepTree());
			} else {
				while (!model.searchStep());
			}
		}
	}

	update();
});

let prevTouchX = 0, prevTouchY = 0;

canvas.addEventListener("touchstart", ev => {
	if (ev.touches.length === 1) {
		prevTouchX = ev.touches[0].clientX;
		prevTouchY = ev.touches[0].clientY;
	}
});

canvas.addEventListener("touchend", ev => {
	if (ev.touches.length === 1) {
		prevTouchX = ev.touches[0].clientX;
		prevTouchY = ev.touches[0].clientY;
	}
});

canvas.addEventListener("touchmove", ev => {
	if (ev.touches.length === 1) {
		const x = ev.touches[0].clientX;
		const y = ev.touches[0].clientY;

		scene.rotation.x += (y - prevTouchY) * 8 / canvas.clientHeight;
		scene.rotation.y += (x - prevTouchX) * 8 / canvas.clientHeight;

		prevTouchX = x;
		prevTouchY = y;

		update();
	}

	ev.preventDefault();
}, { passive: false });

canvas.addEventListener("wheel", ev => {
	if (ev.deltaMode === 0) {
		scene.rotation.x -= ev.deltaY / 256;
		scene.rotation.y -= ev.deltaX / 256;
	}
	else if (ev.deltaMode === 1) {
		scene.rotation.x -= ev.deltaY / 8;
		scene.rotation.y -= ev.deltaX / 8;
	}

	update();

	ev.preventDefault();
});
