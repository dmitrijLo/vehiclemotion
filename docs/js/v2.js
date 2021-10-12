/**
 * @author Stefan Goessner (c) 2013-2021
 * @license MIT Licence (MIT)
 */
/* jshint -W014 */
"use strict";

function v2(v) {
    if (Object.getPrototypeOf(v) === v2.prototype)  // shortcut return ...
        return v;
    else {
        const {x, y, r, w, cw, sw, ...rest} = v;
        let X = 0, Y = 0;
        if (x !== undefined && y !== undefined) { // cartesian coords (primary) .. !
            X = x;
            Y = y;
        }
        else if (r !== undefined && w !== undefined) { // polar coords .. !
            X = r * Math.cos(w);
            Y = r * Math.sin(w);
        }
        else if (r !== undefined && cw !== undefined && sw !== undefined) {
            X = r * cw;
            Y = r * sw;
        }
        else if (r !== undefined) { // to complete .. !
            X = r;
            Y = 0;
        }
        else if (w !== undefined) { // to complete .. !
            X = Math.cos(w);
            Y = Math.sin(w);
        }
        else if (x !== undefined) { // to complete .. ! 
            X = x;
            Y = 0;
        }
        else if (y !== undefined) { // to complete .. !
            X = 0;
            Y = y;
        }
        return v2.create({x:X,y:Y,...rest});
    }
}
v2.prototype = {
    set(v) { 
        this.x = v.x; this.y = v.y;
        return this;
    },
    /**
     *  immutable object
     *  {object} v2
     */
    get const() { 
        return Object.freeze(this); 
    },
    /**
     * @type {boolean}
     */
    get isZero() { 
        return Math.abs(this.x) < Number.EPSILON && Math.abs(this.y) < Number.EPSILON; 
    },
    /**
     * magnitude / length
     * @type {number}
     */
    get r() { return Math.hypot(this.x,this.y); },
    set r(q) { const r = Math.hypot(this.x,this.y); this.x *= q/r; this.y *= q/r; },
    /**
     * angle in [rad]
     * @type {number}
     */
    get w() { return Math.atan2(this.y,this.x); },
    set w(q) { const r = Math.hypot(this.x,this.y); this.x = r*Math.cos(q); this.y = r*Math.sin(q); },
    /**
     * provide magnitude squared.
     * @type {number}
     */
    get sqr() { return this.x**2 + this.y**2; },
    /**
     * provide copy of this vector.
     * @type {object} v2
     */
    get cpy() { return v2.create({x:this.x,y:this.y}); },
    /**
     * provide tilde (orthogonalized) vector
     * @type {object} v2
     */
    get tilde() { return v2.create({x:-this.y,y:this.x}); },
    get ort() { return v2.create({x:-this.y,y:this.x}); },
    get skew() { return v2.create({x:-this.y,y:this.x}); },
    /**
     * negated vector
     * @type {object} v2
     */
    get neg()  { return v2.create({x:-this.x,y:-this.y}); },
    /**
     * unit (normalized) vector
     * @type {object} v2
     */                                         // r not defined => this.r
    get unit() { const l = Math.hypot(this.x,this.y); return v2.create({x:this.x/this.r, y:this.y/l}); },
    /**
     * inverted vector
     * @type {object} v2
     */
    get inv()  { const rr = this.x**2 + this.y**2; return v2.create({x:this.x/rr, y:this.y/rr}); },
    /**
     * provide plain (prototype-free) vector
     * @type {object} v2
     */
    get plain()  { return {x:this.x, y:this.y}; },
    /**
     * dot (inner) product
     * @returns {number}
     * @param {object} v - other vector.
     */
    dot(v) {
        v = v2(v);
        return this.x*v.x + this.y*v.y;
    },
    /**
     * symplectic inner product
     * @returns {number}
     * @param {object} v - other vector.
     */
    symp(v) {
        v = v2(v);
        return this.x*v.y - this.y*v.x;
    },
    /**
     * vector addition
     * @returns {object} v2
     * @param {object} v2 - other vector.
     */
    add(v) {
        v = v2(v);
        return v2.create({x:this.x+v.x,y:this.y+v.y});
    },
    /**
     * vector subtraction
     * @returns {object} v2
     * @param {object} v2 - other vector.
     */
    sub(v) {
        v = v2(v);
        return v2.create({x:this.x-v.x,y:this.y-v.y});
    },
    /**
     * vector scaling
     * @returns {object} v2
     * @param {number} s - scale factor.
     */
    scl(s) { 
        return v2.create({x:this.x*s, y:this.y*s});
    },
    /**
     * vector rotation
     * @returns {object} v2
     * @param {number} s - scale factor.
     */
    rot(w) { 
        let cw=Math.cos(w), sw=Math.sin(w); 
        return v2.create({x:cw*this.x-sw*this.y,y:sw*this.x+cw*this.y}); 
    },
    /**
     * vector similarity transform
     * @returns {object} v2
     * @param {number} lam - first transform factor.
     * @param {number} mu  - second transform factor.
     */
    simtrf(lam,mu) {
        return v2.create({ x: lam*this.x - mu*this.y, y: lam*this.y + mu*this.x});
    },
    /**
     * inplace tilde (orthogonalized) vector
     * @returns {object} this
     */
    itilde()  { let x = -this.y; this.y = this.x; this.x = x; return this; },
    /**
     * inplace invert vector
     * @returns {object} this
     */
    ineg()  { this.x = -this.x; this.y = -this.y; return this; },
    /**
     * inplace normalize (to unit) vector
     * @returns {object} this
     */
    iunit()  { const r = Math.hypot(this.x,this.y); if (r) { this.x /= r; this.y /= r; } return this; },
    /**
     * inplace invert vector
     * @returns {object} this
     */
    iinv()  { const rr = this.x**2 + this.y**2; if (rr) { this.x /= rr; this.y /= rr; } return this; },
    /**
     * inplace rotate vector
     * @returns {object} this
     * @param {number} w - rotation angle [rad].
     */
    irot(w) { 
        const cw = Math.cos(w), 
        sw = Math.sin(w), 
        // cx/sy are not defined
        //x = this.cx * cw - this.sy * sw;
        x = this.x * cw - this.y * sw;
        this.y = this.x * sw + this.y * cw; 
        this.x = x;
        return this; },
    /**
     * inplace copy another vector (use v2.set instead !)
     * @returns {object} this
     * @param {object} v - source vector.
     */
    icpy(v) { this.x = v.x; this.y = v.y; return this; },
    /**
     * inplace scale vector
     * @returns {object} this
     * @param {number} s - scale factor.
     */
    iscl(s) { this.x *= s; this.y *= s; return this; },
    /**
     * inplace add vector
     * @returns {object} this
     * @param {object} v - other vector.
     */
    iadd(v) { this.x += v.x; this.y += v.y; return this; },
    /**
     * inplace subtract vector
     * @returns {object} this
     * @param {object} v - other vector.
     */
    isub(v) { this.x -= v.x; this.y -= v.y; return this; },
    /**
     * inplace similarity transform vector
     * @returns {object} this
     * @param {number} lam - first transform factor.
     * @param {number} mu  - second transform factor.
     */
    isimtrf(lam,mu) { 
        const x = lam*this.x - mu*this.y; 
        this.y  = lam*this.y + mu*this.x; 
        this.x  = x; 
        return this;
    },
    /**
     * constraining methods
     */
    /**
     * Keep absolute angle to vector `v` constant.
     * @returns {object} this
     * @param {object} v - other vector.
     * @param {number} w  - angle in radians.
     */
    cstrAng(v,w) {
        let ux = this.x - v.x, uy = this.y - v.y, r = Math.hypot(ux,uy);
        this.x = v.x + r*Math.cos(w); 
        this.y = v.y + r*Math.sin(w);
        return this;
    },
    /**
     * formatted output string of vector '{x,y,r,w}'.
     * @returns {string}

     */
    toString() { return `{x:${this.x},y:${this.y},r:${this.r},w:${180/Math.PI*this.w}}`; }
}

// redundancies ...
v2.prototype.perp = v2.prototype.symp;

// statics ...
v2.create = function create(v) { return Object.create(v2.prototype, Object.getOwnPropertyDescriptors(v)); }

v2.zero  = v2.create({x:0,y:0}).const
v2.xunit = v2.create({x:1,y:0}).const
v2.yunit = v2.create({x:0,y:1}).const
v2.cpy = function(v) { return v2.create({x:v.x, y:v.y}) }, 
v2.ref = function(v) { return v2.create({get x(){return v.x}, get y(){return v.y},set x(q){v.x=q}, set y(q){v.y=q}}); }
v2.sum = function(a,b) { return v2({x:a.x+b.x, y:a.y+b.y}); }
v2.dif = function(a,b) { return v2({x:a.x-b.x, y:a.y-b.y}); }
v2.angle = function(u,v) {
    var t;
    return v ? Math.atan2(Math.abs(t = u.x*v.y - u.y*v.x) < v2.EPS ? 0 : t, u.x*v.x + u.y*v.y)
             : Math.atan2(u.y, u.x);
 };

v2.case1 = function(a,b,c) {
    a.icpy(b.add(c).ineg);
}
v2.case2 = function case2(a,b,c,sgn) {
    let ar = a.r, ca = a.cw, sa = a.sw, br = b.r, cb, sb, cx = c.x, cy = c.y,
        sqr = br*br - (ca*cy - sa*cx)**2;
    a.r = (-cx*ca - cy*sa + (sgn||1)*Math.sqrt(sqr > 0 ? sqr : 0));
    sb = -(ar*sa + cy)/br, cb = -(ar*ca + cx)/br;
    b.cw = cb; b.sw = sb;
}
v2.case3 = function case3(a,b,c) {
    let ca = a.cw, sa = a.sw, 
        cb = b.cw, sb = b.sw,
        den = ca*sb - sa*cb;
    a.r =  (c.y*cb - c.x*sb)/den;
    b.r = -(c.y*ca - c.x*sa)/den;
}
v2.case4 = function case4(a,b,c,sgn) {
    let aa = a.r*a.r, cc = c.r*c.r, aa_cc = aa/cc,
        lam = (aa_cc - b.r*b.r/cc + 1)/2,
        sqr = aa_cc - lam*lam,
        mu = sqr > 0 ? (sgn||1)*Math.sqrt(sqr) : 0,
        cx = c.x, cy = c.y,
        ca = (-lam*cx + mu*cy)/a.r, sa = (-lam*cy - mu*cx)/a.r, 
        cb = (-a.r*ca - cx)/b.r, sb = (-a.r*sa - cy)/b.r;
    a.cw = ca; a.sw = sa;
    b.cw = cb; b.sw = sb;
}
v2.case5 = function case5(a,b,c,sgn) {
    let cc = c.sqr, aa = cc + b.r*b.r, ca, sa;
    a.r = (sgn||1)*Math.sqrt(aa > 0 ? aa : 0);
    ca = (-a.r*c.x - b.r*c.y)/cc; sa = (-a.r*c.y + b.r*c.x)/cc;
    a.cw =  ca; a.sw = sa;
    b.cw = -sa; b.sw = ca;
}

// use it with node.js ... ?
if (typeof module !== 'undefined') module.exports = v2;