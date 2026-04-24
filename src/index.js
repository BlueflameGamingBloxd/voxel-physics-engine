var aabb = require('aabb-3d')
var vec3 = require('gl-vec3')
var sweep = require('voxel-aabb-sweep')

import { RigidBody } from './rigidBody'
export { RigidBody }

var DEBUG = 0
var EPSILON = 1e-6
var MAX_ITERATIONS = 5

export function DefaultOptions() {
    this.airDrag = 0.1
    this.fluidDrag = 0.4
    this.fluidDensity = 2.0
    this.gravity = [0, -10, 0]
    this.minBounceImpulse = .5
    this.maxVelocity = 50
    this.subSteps = 1
    this.penetrationCorrection = 0.2
}

export function Physics(opts, testSolid, testFluid, testSubmerged = null) {
    opts = Object.assign(new DefaultOptions(), opts)

    this.gravity = opts.gravity || [0, -10, 0]
    this.airDrag = opts.airDrag || 0.1
    this.fluidDensity = opts.fluidDensity || 2.0
    this.fluidDrag = opts.fluidDrag || 0.4
    this.minBounceImpulse = opts.minBounceImpulse
    this.maxVelocity = opts.maxVelocity
    this.penetrationCorrection = opts.penetrationCorrection
    this.subSteps = opts.subSteps
    this.bodies = []
    this.terrainEvents = opts.terrainEvents || false
    this.debugMode = opts.debugMode || false
    
    this.collisionCache = new Map()
    this.cacheTimeout = 100
    
    this.testSolid = testSolid
    this.testFluid = testFluid
    this.testSubmerged = testSubmerged
    
    this.onBodyAdded = null
    this.onBodyRemoved = null
    this.onCollision = null
    this.onFluidEnter = null
    this.onFluidExit = null
    
    this.debugCollisions = []
}

Physics.prototype.addBody = function (_aabb, mass, friction, restitution, gravMult, onCollide, customData = null) {
    _aabb = _aabb || new aabb([0, 0, 0], [1, 1, 1])
    if (typeof mass == 'undefined') mass = 1
    if (typeof friction == 'undefined') friction = 1
    if (typeof restitution == 'undefined') restitution = 0
    if (typeof gravMult == 'undefined') gravMult = 1
    
    var b = new RigidBody(_aabb, mass, friction, restitution, gravMult, onCollide)
    b.customData = customData
    b.fluidExposure = 0
    b.collisionNormal = vec3.create()
    b.groundContact = false
    b.airTime = 0
    
    this.bodies.push(b)
    
    if (this.onBodyAdded) this.onBodyAdded(b)
    return b
}

Physics.prototype.removeBody = function (b) {
    var i = this.bodies.indexOf(b)
    if (i < 0) return undefined
    
    this.bodies.splice(i, 1)
    
    if (this.onBodyRemoved) this.onBodyRemoved(b)
    
    for (let [key, value] of this.collisionCache) {
        if (value.body === b) this.collisionCache.delete(key)
    }
    
    b.aabb = b.onCollide = b.customData = null
}

Physics.prototype.addBodies = function (bodies) {
    bodies.forEach(body => this.addBody(body.aabb, body.mass, body.friction, 
                                         body.restitution, body.gravMult, body.onCollide, body.customData))
}

Physics.prototype.clearBodies = function () {
    this.bodies.forEach(b => {
        b.aabb = b.onCollide = b.customData = null
    })
    this.bodies = []
    this.collisionCache.clear()
}

Physics.prototype.setDebugMode = function (enabled) {
    this.debugMode = enabled
    if (!enabled) this.debugCollisions = []
}

Physics.prototype.getDebugCollisions = function () {
    return this.debugCollisions
}

var a = vec3.create()
var dv = vec3.create()
var dx = vec3.create()
var impacts = vec3.create()
var oldResting = vec3.create()
var beforePreventFallDx = vec3.create()
var preventFallResting = vec3.create()
var rollbackAabb = new aabb([0, 0, 0], [1, 1, 1])
var rollbackBody = new RigidBody(rollbackAabb, 1, 1, 0, 1, () => {}, false)
var timeStepAccumulator = 0
var fixedTimeStep = 1 / 60

Physics.prototype.tick = function (dt, useFixedTimestep = true) {
    if (useFixedTimestep) {
        timeStepAccumulator += dt / 1000
        let steps = 0
        
        while (timeStepAccumulator >= fixedTimeStep && steps < MAX_ITERATIONS) {
            this.updatePhysics(fixedTimeStep)
            timeStepAccumulator -= fixedTimeStep
            steps++
        }
    } else {
        this.updatePhysics(dt / 1000)
    }
}

Physics.prototype.updatePhysics = function (dt) {
    var noGravity = equals(0, vec3.squaredLength(this.gravity))
    
    this.bodies.forEach(b => this.iterateBody(b, dt, noGravity))
    
    if (this.debugMode) this.debugCollisions = []
}

Physics.prototype.iterateBody = function (b, dt, noGravity) {
    vec3.copy(oldResting, b.resting)
    if (this.isBodyInsideUnloadedBlock && this.isBodyInsideUnloadedBlock(b)) {
        rollbackBody.loadFromCopy(b)
    }

    if (b.mass <= 0) {
        vec3.set(b.velocity, 0, 0, 0)
        vec3.set(b._forces, 0, 0, 0)
        vec3.set(b._impulses, 0, 0, 0)
        return
    }

    var localNoGrav = noGravity || (b.gravityMultiplier === 0)
    if (this.bodyAsleep(b, dt, localNoGrav)) return
    b._sleepFrameCount--

    this.applyFluidForces(b)
    
    var wasOnGround = b.groundContact
    b.groundContact = false

    vec3.scale(a, b._forces, 1 / b.mass)
    vec3.scaleAndAdd(a, a, this.gravity, b.gravityMultiplier)
    vec3.scale(dv, b._impulses, 1 / b.mass)
    vec3.scaleAndAdd(dv, dv, a, dt)
    vec3.add(b.velocity, b.velocity, dv)
    
    if (vec3.squaredLength(b.velocity) > this.maxVelocity * this.maxVelocity) {
        vec3.normalize(b.velocity, b.velocity)
        vec3.scale(b.velocity, b.velocity, this.maxVelocity)
    }

    if (b.friction) {
        this.applyFrictionByAxis(0, b, dv, dt)
        this.applyFrictionByAxis(1, b, dv, dt)
        this.applyFrictionByAxis(2, b, dv, dt)
    }

    var drag = (b.airDrag >= 0) ? b.airDrag : this.airDrag
    if (b.inFluid) {
        drag = (b.fluidDrag >= 0) ? b.fluidDrag : this.fluidDrag
        drag *= 1 - Math.pow(1 - b.ratioInFluid, 2)
    }
    var mult = Math.max(1 - drag * dt / b.mass, 0)
    vec3.scale(b.velocity, b.velocity, mult)

    vec3.scale(dx, b.velocity, dt)
    
    vec3.set(b._forces, 0, 0, 0)
    vec3.set(b._impulses, 0, 0, 0)

    if (b.autoStep) {
        cloneAABB(this.tmpBox, b.aabb)
    }

    vec3.copy(beforePreventFallDx, dx)
    vec3.set(preventFallResting, 0, 0, 0)
    if (b.preventFallOffEdge) {
        this.tryPreventFallOffEdge(b, dx, preventFallResting)
    }

    this.processCollisions(b.aabb, dx, b.resting)
    
    if (this.penetrationCorrection > 0) {
        this.correctPenetration(b)
    }

    if (b.autoStep) {
        this.tryAutoStepping(b, this.tmpBox, beforePreventFallDx)
    }

    b.groundContact = (b.resting[1] !== 0)
    
    if (b.groundContact) {
        b.airTime = 0
    } else {
        b.airTime += dt
    }

    this.handleCollisionImpacts(b, oldResting, impacts)

    var vsq = vec3.squaredLength(b.velocity)
    if (vsq > 1e-5) b._markActive()

    if (this.isBodyInsideUnloadedBlock && this.isBodyInsideUnloadedBlock(b)) {
        b.loadFromCopy(rollbackBody)
    }

    if (this.onFluidEnter || this.onFluidExit) {
        if (b.inFluid && !b.wasInFluid && this.onFluidEnter) this.onFluidEnter(b)
        if (!b.inFluid && b.wasInFluid && this.onFluidExit) this.onFluidExit(b)
        b.wasInFluid = b.inFluid
    }
}

var _fluidVec = vec3.create()
var _corners = [vec3.create(), vec3.create(), vec3.create(), vec3.create(), 
                vec3.create(), vec3.create(), vec3.create(), vec3.create()]

Physics.prototype.applyFluidForces = function (body) {
    var box = body.aabb
    var submerged = 0
    var totalPoints = 0
    
    for (var i = 0; i < 8; i++) {
        var corner = _corners[i]
        corner[0] = (i & 1) ? box.max[0] : box.base[0]
        corner[1] = (i & 2) ? box.max[1] : box.base[1]
        corner[2] = (i & 4) ? box.max[2] : box.base[2]
        
        var cx = Math.floor(corner[0])
        var cy = Math.floor(corner[1])
        var cz = Math.floor(corner[2])
        
        if (this.testFluid(cx, cy, cz)) {
            submerged++
        }
        totalPoints++
    }
    
    var ratioInFluid = submerged / totalPoints
    var vol = box.vec[0] * box.vec[1] * box.vec[2]
    var displaced = vol * ratioInFluid
    
    var f = _fluidVec
    vec3.scale(f, this.gravity, -this.fluidDensity * displaced)
    body.applyForce(f)
    
    body.inFluid = (ratioInFluid > 0)
    body.ratioInFluid = ratioInFluid
}

var lateralVel = vec3.create()

Physics.prototype.applyFrictionByAxis = function (axis, body, dvel, dt) {
    var restDir = body.resting[axis]
    var vNormal = dvel[axis]
    
    if (!body.alwaysApplyHorizFriction || axis !== 1) {
        if (restDir === 0) return
        if (restDir * vNormal <= 0) return
    }

    vec3.copy(lateralVel, body.velocity)
    lateralVel[axis] = 0
    var vCurr = vec3.length(lateralVel)
    if (equals(vCurr, 0)) return

    var frictionCoeff = body.friction
    if (this.testSolid && axis === 1 && restDir !== 0) {
        var groundX = Math.floor(body.aabb.base[0] + body.aabb.vec[0] / 2)
        var groundZ = Math.floor(body.aabb.base[2] + body.aabb.vec[2] / 2)
        var groundY = Math.floor(body.aabb.base[1] - 0.1)
    }
    
    var dvMax = Math.abs(frictionCoeff * this.gravity[axis] * dt)
    var scaler = (vCurr > dvMax) ? (vCurr - dvMax) / vCurr : 0
    body.velocity[(axis + 1) % 3] *= scaler
    body.velocity[(axis + 2) % 3] *= scaler
}

var collisionKey = [0, 0, 0]

Physics.prototype.processCollisions = function (box, velocity, resting) {
    vec3.set(resting, 0, 0, 0)
    
    if (this.collisionCache && vec3.squaredLength(velocity) < EPSILON) {
        var key = `${Math.floor(box.base[0])},${Math.floor(box.base[1])},${Math.floor(box.base[2])}`
        if (this.collisionCache.has(key) && Date.now() - this.collisionCache.get(key).timestamp < this.cacheTimeout) {
            var cached = this.collisionCache.get(key)
            if (cached.resting) vec3.copy(resting, cached.resting)
            return
        }
    }
    
    var result = sweep(this.testSolid, box, velocity, (dist, axis, dir, vec) => {
        resting[axis] = dir
        vec[axis] = 0
    })
    
    if (vec3.squaredLength(velocity) < EPSILON && this.collisionCache) {
        var cacheKey = `${Math.floor(box.base[0])},${Math.floor(box.base[1])},${Math.floor(box.base[2])}`
        this.collisionCache.set(cacheKey, {
            resting: vec3.clone(resting),
            timestamp: Date.now()
        })
    }
    
    return result
}

Physics.prototype.correctPenetration = function (body) {
    var box = body.aabb
    var base = box.base
    var max = box.max
    
    for (var i = 0; i < 8; i++) {
        var x = (i & 1) ? max[0] - EPSILON : base[0] + EPSILON
        var y = (i & 2) ? max[1] - EPSILON : base[1] + EPSILON
        var z = (i & 4) ? max[2] - EPSILON : base[2] + EPSILON
        
        var cx = Math.floor(x)
        var cy = Math.floor(y)
        var cz = Math.floor(z)
        
        if (this.testSolid(cx, cy, cz)) {
            var dx = Math.min(Math.abs(x - (cx + 0.5)), Math.abs(x - (cx + 0.5))) 
            var dy = Math.min(Math.abs(y - (cy + 0.5)), Math.abs(y - (cy + 0.5)))
            var dz = Math.min(Math.abs(z - (cz + 0.5)), Math.abs(z - (cz + 0.5)))
            
            if (dy <= dx && dy <= dz) {
                body.aabb.base[1] += (y - (cy + 0.5)) * this.penetrationCorrection
                body.aabb.max[1] += (y - (cy + 0.5)) * this.penetrationCorrection
            } else if (dx <= dy && dx <= dz) {
                body.aabb.base[0] += (x - (cx + 0.5)) * this.penetrationCorrection
                body.aabb.max[0] += (x - (cx + 0.5)) * this.penetrationCorrection
            } else {
                body.aabb.base[2] += (z - (cz + 0.5)) * this.penetrationCorrection
                body.aabb.max[2] += (z - (cz + 0.5)) * this.penetrationCorrection
            }
        }
    }
}

Physics.prototype.handleCollisionImpacts = function (b, oldResting, impacts) {
    for (var i = 0; i < 3; ++i) {
        b.resting[i] = b.resting[i] || this.preventFallResting?.[i] || 0
        impacts[i] = 0
        
        if (b.resting[i]) {
            if (!oldResting[i]) impacts[i] = -b.velocity[i]
            b.velocity[i] = 0
        }
    }
    
    var mag = vec3.length(impacts)
    if (mag > EPSILON) {
        vec3.scale(impacts, impacts, b.mass)
        
        vec3.normalize(b.collisionNormal, impacts)
        
        if (b.onCollide) b.onCollide(impacts)
        
        if (b.restitution > 0 && mag > this.minBounceImpulse) {
            var effectiveRestitution = b.restitution
            if (impacts[1] < 0 && b.resting[1]) {
                effectiveRestitution *= 0.8
            }
            vec3.scale(impacts, impacts, effectiveRestitution)
            b.applyImpulse(impacts)
        }
        
        if (this.onCollision) this.onCollision(b, impacts)
        
        if (this.debugMode) {
            this.debugCollisions.push({
                body: b,
                impulse: vec3.clone(impacts),
                position: vec3.clone(b.aabb.base),
                time: Date.now()
            })
            while (this.debugCollisions.length > 100) this.debugCollisions.shift()
        }
    }
}

Physics.prototype.tryAutoStepping = function (b, oldBox, dx) {
    if (b.resting[1] >= 0 && !b.inFluid) return

    var xBlocked = (b.resting[0] !== 0)
    var zBlocked = (b.resting[2] !== 0)
    if (!(xBlocked || zBlocked)) return

    var ratio = Math.abs(dx[0] / (dx[2] + EPSILON))
    var cutoff = 4
    if (!xBlocked && ratio > cutoff) return
    if (!zBlocked && ratio < 1 / cutoff) return

    vec3.add(this.targetPos, oldBox.base, dx)
    
    var getVoxels = this.testSolid
    sweep(getVoxels, oldBox, dx, (dist, axis, dir, vec) => {
        if (axis === 1) vec[axis] = 0
        else return true
    })

    var y = b.aabb.base[1]
    var ydist = Math.floor(y + 1.001) - y
    
    var maxStepHeight = b.maxStepHeight || 1.0
    var actualStep = Math.min(ydist, maxStepHeight)
    vec3.set(this.upvec, 0, actualStep, 0)
    
    var collided = false
    sweep(getVoxels, oldBox, this.upvec, (dist, axis, dir, vec) => {
        collided = true
        return true
    })
    if (collided) return

    vec3.subtract(this.leftover, this.targetPos, oldBox.base)
    this.leftover[1] = 0
    this.processCollisions(oldBox, this.leftover, this.tmpResting)

    var xMovedToTarget = equals(oldBox.base[0], this.targetPos[0])
    var zMovedToTarget = equals(oldBox.base[2], this.targetPos[2])
    
    if (xBlocked && !xMovedToTarget && (!zMovedToTarget || !zBlocked)) return
    if (zBlocked && (!xMovedToTarget || !xBlocked) && !zMovedToTarget) return

    var moveIsBad = !this.hasGroundContact(oldBox.base[0] + 1e-5, oldBox.base[1], oldBox.base[2] + 1e-5)
    moveIsBad = moveIsBad && !this.hasGroundContact(oldBox.max[0] - 1e-5, oldBox.base[1], oldBox.base[2] + 1e-5)
    moveIsBad = moveIsBad && !this.hasGroundContact(oldBox.base[0] + 1e-5, oldBox.base[1], oldBox.max[2] - 1e-5)
    moveIsBad = moveIsBad && !this.hasGroundContact(oldBox.max[0] - 1e-5, oldBox.base[1], oldBox.max[2] - 1e-5)
    
    if (moveIsBad) return
    
    cloneAABB(b.aabb, oldBox)
    b.resting[0] = this.tmpResting[0]
    b.resting[2] = this.tmpResting[2]
    if (b.onStep) b.onStep()
}

Physics.prototype.hasGroundContact = function (x, y, z) {
    for (var offset = 0; offset <= 1.0; offset += 0.5) {
        if (this.testSolid(Math.floor(x), Math.floor(y - offset), Math.floor(z))) {
            return true
        }
    }
    return false
}

Physics.prototype.bodyAsleep = function (body, dt, noGravity) {
    if (body._sleepFrameCount > 0) return false
    if (noGravity) return true
    
    var vel = vec3.squaredLength(body.velocity)
    if (vel > 0.01) return false
    
    var isResting = false
    var gmult = 0.5 * dt * dt * body.gravityMultiplier
    vec3.scale(sleepVec, this.gravity, gmult)
    
    sweep(this.testSolid, body.aabb, sleepVec, () => {
        isResting = true
        return true
    }, true)
    
    return isResting && vel < 0.001
}

function equals(a, b) { return Math.abs(a - b) < EPSILON }

function cloneAABB(tgt, src) {
    for (var i = 0; i < 3; i++) {
        tgt.base[i] = src.base[i]
        tgt.max[i] = src.max[i]
        tgt.vec[i] = src.vec[i]
    }
}

var sanityCheck = function (v) { }
if (DEBUG) sanityCheck = function (v) {
    if (isNaN(vec3.length(v))) throw 'Vector with NAN: ' + v
}

Physics.prototype.tmpBox = new aabb([], [])
Physics.prototype.tmpResting = vec3.create()
Physics.prototype.targetPos = vec3.create()
Physics.prototype.upvec = vec3.create()
Physics.prototype.leftover = vec3.create()
Physics.prototype.preventFallResting = preventFallResting
