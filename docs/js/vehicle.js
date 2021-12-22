/** 
 * @requires v2.js
 * @requires canvasInteractor.js
 * @requires g2.js
 * @requires g2.selector.js
*/

// Fahrzeugbeispiel: PKW: Honda-Civic bj2009
// Abmessungen: Länge: 4.270mm, Breite: 1.785mm, Radstand: 2.620mm
// Reifen: 205/55R16 91 H => Breite = 205mm, Durchmesser 16Zoll = 406,4mm

g2.prototype.wheel = function(args) { return this.addCommand({c:'wheel',a:args}); }
g2.prototype.wheel.prototype = g2.mix(g2.prototype.rec.prototype, {
    g2() {
        const {x,y,b,h,w,ls='black',fs='#ccc',sh} = this;
        return g2().use({ 
            grp: g2().rec({x: -b/2, y: -h/2, b,h,fs})
                     .lin({x1: -b/2 * 0.75, y1: h/2 * 0.85, x2: b/2 * 0.75, y2: h/2 * 0.85})
                     .lin({x1: -b/2 * 0.75, y1: h/2 * 0.65, x2: b/2 * 0.75, y2: h/2 * 0.65})
                     .lin({x1: -b/2 * 0.75, y1: h/2 * 0.3, x2: b/2 * 0.75, y2: h/2 * 0.3})
                     .lin({x1: -b/2 * 0.75, y1: -h/2 * 0.85, x2: b/2 * 0.75, y2: -h/2 * 0.85})
                     .lin({x1: -b/2 * 0.75, y1: -h/2 * 0.65, x2: b/2 * 0.75, y2: -h/2 * 0.65})
                     .lin({x1: -b/2 * 0.75, y1: -h/2 * 0.3, x2: b/2 * 0.75, y2: -h/2 * 0.3}),
            x, y, w: w + Math.PI/2 })
    }
});

const rearAxis = ({l,b,h}) => {
    return g2().ply({pts:[-l/2,0,l/2,0], lw:5, ls:'#808080'}).wheel({x:-l/2, y:0, b, h}).wheel({x:l/2, y:0, b, h});
}

function casteljau(pts,t){
    if(pts.length === 1) { return pts[0]; }
    else{
        let nextPts = [];
        for(let i = 0; i < pts.length - 1; i++){
            const x = (1-t) * pts[i].x + t * pts[i+1].x,
                  y = (1-t) * pts[i].y + t * pts[i+1].y;
            nextPts.push({x:x,y:y});         
        }
        return casteljau(nextPts,t);
    }
}

function bezier(pts){
    const bezr = [];
    for(let t=0; t<=1; t+=0.01){
        t = ~~(t*1000)/1000
        bezr.push(casteljau(pts,t));
    }
    return bezr;
}

const vehicle = {
    create() {
        const self = Object.create(this.prototype);
        self.constructor.apply(self,arguments); 
        return self; 
    },
    prototype: {
        constructor(guidingPoint,drawbar,u){
            this._g = g2();
            this._A = typeof guidingPoint == 'function' ? () => guidingPoint() : guidingPoint; // reference to previous point
            this._B = this.A.add(v2.yunit.scl(drawbar));            
            this._koppel = 0;
            this._C = this.B.add(this.e.neg.scl(this.koppel));
            this._drawbar = drawbar;
            this._u = u;
            this._phi = this.e.w;
            this.handle = this.A.add(this.e.rot(this.gamma).scl(50));
            this._cacheA = [this.A.cpy,this.A.cpy,this.A.cpy];

            this._chassi = { b: 178.5/4, l: this.drawbar/* 427 */, img: "./img/honda.png" }
            this._wheel = { b: 20.5/4, h: 40.64/4 }
            this.dir = 1;
            this._ds = 0;
            this._dt = 1;
            this._tick = 0;
            this._t = 0;
            this._mu = 1;
            this._path = [];
            this._isActive = false;
            this._hasTrailer = false;
            this._traceB = [];
            this._history = []; // bezogen auf die Simulation entlang der Traktrix
            this._diff = 0;
            this.update = this.update.bind(this);
        },
        get A() {return typeof this._A == 'function' ? this._A() : this._A},
        set A(o) {this._A.x = o.x; this._A.y = o.y},
        get B() { return this._B },
        set B(o) { this._B.x = o.x; this._B.y = o.y },
        get BA() { return this.A.add(this.B.neg); },
        get C() { return this._C },
        set C(o) { this._C.x = o.x; this._C.y = o.y },
        // Richtungsvektoren
        get e() { return this.BA.unit },
        get u() { return this._u()},
        set u(fn) { this._u = fn;},

        // Geometrie
        get drawbar() { return this._drawbar; },
        set drawbar(int) { this._drawbar = int; },
        get koppel() { return this._koppel },
        set koppel(int) { return this._koppel = int },
        get h() { return this.drawbar/Math.tan(this.gamma); },
        get r() { return this.drawbar/Math.sin(this.gamma); },
        get rho() { return this.getCurvature(); },
        

        get v(){ return this.dir * this.cacheA[2].sub(this.cacheA[1]).scl(1/this._dt).r},
    
        // Pole der ebenen Bewegung
        get PA() { return this.u.tilde.neg.scl(this.r); },
        get PW() { return this.e.scl(this.r/Math.sin(this.gamma) * (1 - this.r/this.rho))},
        get PT() { return this.e.tilde.scl(-this.r/Math.cos(this.gamma)); },
        get P() { return this.A.add(this.PA.neg); },// Momentanpol
        get W() { return this.P.add(this.PW); }, // Wendepol
        get T() { return this.P.add(this.PT); }, // Tangentialpol
        // Wende- und Tangentialkreismittelpunkt
        get inflCir() { return this.P.add(this.PW.scl(0.5)); },
        get tangCir() { return this.P.add(this.PT.scl(0.5)); },
        get traceB() { return this._traceB },
        set traceB(o) { this._traceB.push({ x: o.x, y: o.y }) },
        
        // Winkel, Winkelgeschwindigkeit und -beschleunigung
        get omega() { return this.v/this.drawbar * Math.sin(this.gamma); },
        get dotOmega() { return (this.v**2)/this.drawbar**2 * (this.r/this.rho - 1) * Math.sin(this.gamma) * Math.cos(this.gamma); },

        //get phi() { return this.omega * (1/60) + (this.dotOmega * (1/60)**2)/2; },
        get gamma() { return v2.angle(this.e, this.u) },
        //get gamma() { return typeof this._gamma === 'function' ? this._gamma(): this._gamma; },
        //set gamma(rad) {this._gamma = rad; },

        get cacheA() {return this._cacheA; },
        set cacheA(o) { this._cacheA.push(o); },

        getDimensions(){
            let A = this.B.add(this.e.tilde.scl(this._chassi.b/2)),
                B = this.B.add(this.e.tilde.neg.scl(this._chassi.b/2)),
                C = B.add(this.e.scl(this._chassi.l)),
                D = A.add(this.e.scl(this._chassi.l));
                return [A,B,C,D];
        },
        getCurvature(){
            const [A,B,C] = this.cacheA;
            const AB = A.add(B),
                  BC = B.add(C),
                  //lambda = 0.5 * v2.dot(v2.dif(AB,BC),AB) / v2.perp(BC,AB),
                  mu = -0.5 * AB.sub(BC).dot(BC) / AB.perp(BC),
                  center = AB.scl(0.5).add(AB.tilde.scl(mu));
            return !isNaN(center.r) ? center.r : Infinity;
        },
        update(dt){
            this._tick += dt;
            if(this.A.sub(this.cacheA[2]).r === 0) return            
            
            this.cacheA = this.A.cpy;
            if(this.cacheA.length > 3) this.cacheA.shift();
            
            
            
            this.incrementPhi(dt);
            this.C = this.B.add(this.e.neg.scl(this.koppel));
            const ephi = v2({x: Math.cos(this._phi), y: Math.sin(this._phi) });
            this.B = this.A.add(ephi.neg.scl(this.drawbar))
            //if(this.traceB.length > 500) this.traceB.shift();
            if(this._tick > 1/30) {
                this._tick = 0;
                this.traceB = this.B;
            }
        },
        drivePath(time){
            const dt = 1/60, step = 1 / (time/dt);
            this._t += step;
            const nextPtA = v2(casteljau(window.world._bezierController, this._t));
            this.v = nextPtA.sub(this.A).r * 60;
            this.gamma = nextPtA.sub(this.A).w - this.e.w;
            this.incrementPhi(dt);
            this.gamma = nextPtA.sub(this.A).w - this.e.w;
            this.A = v2(casteljau(window.world._bezierController, this._t));
            this.update();
            //this._path.shift();
        },
        driveForward(){ // Simulation der Vorwärtsfahrt oder entlang linearer Traktrix
            this.dt = 1;
            const ds = this._mu * this.drawbar;
            let path = this.drawbar;
            this.cacheA[1] = this.A.sub({x:ds,y:0})
            //this.v = ds;
            //this._phi += this.omega*dt + 0.5*this.dotOmega*dt**2;
            //this.gamma = -this._phi;
            //this._history.push([his.A.cpy, this.B.cpy]);
            while(path > 0){
                path -= ds;
                this.incrementPhi(this.dt);
                this.A.iadd({x: ds, y:0})
                this.cacheA = this.A.cpy;
                if(this.cacheA.length > 3) this.cacheA.shift();
                const ephi = v2({x: Math.cos(this._phi), y: Math.sin(this._phi) });
                this.B = this.A.add(ephi.neg.scl(this.drawbar))
                this.handle = this.A.add(this.u.scl(50));
                this._history.push([this.A.cpy, this.B.cpy/* , phi:this._phi */]);
            }
        },
        correctHandle(){ this.A.add(this.handle.neg).r < 20 ? this.handle = this.A.add(this.u.scl(50)) : null;},
        round(num,digits){ return Math.floor(num * 10**digits)/10**digits;},
        toRad(deg){ return deg === 0 ? 0.001*Math.PI/180 : deg*Math.PI/180},
        incrementPhi(dt){this._phi += this.omega*dt + 0.5*this.dotOmega*dt**2; },
        decrementPhi(dt){this._phi -= this.omega*dt + 0.5*this.dotOmega*dt**2; },
    }
}


const world = {
    create() {
        const self = Object.create(this.prototype);
        self.constructor.apply(self,arguments); 
        return self; 
    },
    prototype: {
        constructor(ctx,{x,y}){
            this._origin = {x:x,y:y,cartesian:true};
            this._ctx = ctx;
            this._interactor = canvasInteractor.create(this._ctx, {x,y,cartesian} = this._origin );
            this._selector = g2.selector(this._interactor.evt);
            this._g = g2().clr().view(this._interactor.view).grid({color: "grey", size: 20});
            

            this.vehicle = vehicle.create(v2({x:0,y:0}),50, () => v2({x:1,y:0}) );
            this._trailers = [this.vehicle];
            this._nodes = [this.vehicle.A, this.vehicle.B];
            this._history = [];
            this._vehicleHistory = g2().view(this._interactor.view)
            
            this._trailerLength = 50;
            this._scale = 1;
            this._tick = 0;
            this._bezierController = [this.vehicle.A.cpy];
            this._path = [];
            this._isDirty = false;
            this._showTrace = true;
            this._isActive = false;
            this._showBody = false;
            this._showTractrix = false;
            this._showHistory = false;
            this._showBackground = false;
            this.init();
        },

        get scale(){return this._scale;},
        //get A() {return this._A},
        //set A(o) {this._A.x = o.x; this._A.y = o.y},
        get u() { return this._u; },
        set u(o) {this._u.x = o.x; this._u.y = o.y},
        get trailers() { return this._trailers; },
        set trailers(o) { this._trailers.push(o); },
        get nodes() { return this._nodes; },
        set nodes(o) { this._nodes.push(o); },

        init(){
            this._g.img({uri: () => this._showBackground ? './img/Straßwalchen_kreisverkehr.jpg' : './img/pixel.png' ,b:2000,h:1500,x: -1200,y:-1000, scl:2.5})
            .use({grp: () => {
                let g = g2();
                if(this._showHistory) for(let nodes of this._history.flat()) g.ply({pts: () => nodes, ls:'rgba(47,79,79,0)', fs:'rgba(47,79,79,0.25)'});
                return g;
            }})
            .use({grp: () => this.vehicle._g})
                    .img({uri: () => this._showBody ? this.vehicle._chassi.img : './img/pixel.png', xoff: -170, yoff: -180, x: () => this.vehicle.B.x, y: () => this.vehicle.B.y, scl:0.65, w: () => this.vehicle.e.w})
                    .use({grp: () => this._showTractrix ? g2().ply({pts: this.tractrix(), lw: 5, ls: 'Crimson'})
                                                              .vec({p1:{x:0,y:0}, p2: {x:1000,y:0}, lw:2, label:{str:'x', loc:0.95}})
                                                              .vec({p1:{x:0,y:0}, p2: {x:0,y:300},lw:2, label:{str:'y', off:-5,loc:0.9}}) : g2()})
                   .ply({pts:() => this.nodes,lw:5,ls:'#808080'})
                   .ply({pts: () => {
                        if(this._showTrace) return this.vehicle.traceB;
                    },lw:2,ls:'Black'})
                    .ply({pts: () => this._path,lw:2,ls:'darkblue'})
                    .use({grp: () => {
                        let g = g2();
                        for(let trailer of this.trailers){
                            g.use({ grp: () => rearAxis({l:trailer._chassi.b,...trailer._wheel}), p: () => trailer.B, w: () => trailer.e.tilde.w })
                        }
                        return g;
                    }})
                    .use({grp: () => {
                        let g = g2();
                        for(let node of this.nodes) g.nod({p: () => node/* , fs:'red' */});
                        return g;
                    }})
                    .vec({p1: () => this.vehicle.A, p2: () => this.vehicle.handle, lw: 2.5 })
                    
                    //.img({uri:truck, x: () => this.A.x, y: () => this.A.y, scl:0.4, xoff: -145,yoff:-50, w: () => this.linkages[0].e.tilde.w})
                    .hdl({ p: () => this.vehicle.C, label:'C' })
                    .hdl({ p: () => this.vehicle.handle, label:'Z', r: () => this._interactor.view.scl < 0.5 ? 15 : 5 })
                    .wheel({p: () => this.vehicle.A, b: 20.5/4,h: 40.64/4, w: () => this.vehicle.e.w + this.vehicle.gamma});
                    
                  
            this._interactor.on('tick', e => { this.ontick(e); })
                            .on('drag', e => { this.ondrag(e); })
                            /* .on('pointerdown', e => {
                                if(!this._selector.selection){
                                    const pt = {x:e.xusr, y:e.yusr};
                                    this._g.hdl(pt);
                                    this._bezierController.push(pt);
                                    this._path = bezier(this._bezierController)
                                }       
                            }) */
                            .on('pointerup', e => {
                                this._path = bezier(this._bezierController)
                            })
                            .on('pan',  (e) => { this.onpan(e); })
                            .on('wheel',  (e) => { this.onwheel(e); })
                            .startTimer();
        },
        createTrailer(){
            const previous = this.trailers[this.trailers.length - 1];
            const next = vehicle.create(previous.C, this._trailerLength, () => previous.e);
            previous._hasTrailer = true;
            this.nodes = previous.C
            this.nodes = next.B;
            this.trailers = next;
        },

        drive(){
            this.A.iadd(this.u.scl(100 * 1/60));
            this.notify();
        },
        tractrix() {
            const drawbar = this.vehicle.drawbar, pts = [];
            for (let i = 0; i <= 5; i += 0.001){
                const t = Math.round(i * 10000)/10000;
                const x = drawbar * (t - Math.tanh(t)), y = drawbar/Math.cosh(t), sx = t * drawbar;
                const b = v2({x: -sx, y:0}).add(v2({x:x,y:y}));
                const phi = 180 - Math.atan2(b.y,b.x) * 180/Math.PI;
                pts.push({x:x,y:y,sx:sx,phi:phi,t:t});
            }
            return pts;
        },
        simulateStep(){
            this.vehicle.driveForward();
            //const itr = this.compareTrajectories();
            //this.vehicle._diff = itr[itr.length - 1].abs;
            this.vehicle._g.del();
            for(let entry of this.vehicle._history){
                this.vehicle._g.ply({pts: () => entry, lw: 2.5, ls:'#003d61' }).nod({p: entry[0]}).nod({p:entry[1]})
            }
            ctrlUpdate();
        },
        compareTrajectories(){ // phi ist in Grad
            const tr1 = this.tractrix(), tr2 = this.vehicle._history;
            const deviation = [];
        
            for(const sxi of tr2){
                for(const sxj of tr1){
                    //console.log(sxi.A.x, sxj.sx)
                   if(sxi.A.x === sxj.sx) {
        
                // Hier to-do: phis sind nicht miteinander verträglich ... das phi der Traktrix begint mit 114° das kann nicht sein...
                        const abs = (- sxi.phi * 180/Math.PI) - sxj.phi, //absoluter Fehler
                              rel = (abs/sxj.phi)*100; //relativer Fehler in Prozent
        
                        deviation.push({abs,rel /* ,phiTr:sxj.phi,phiM: sxi.phi */}); //phiTr => phi Tractrix, phiM => phi Modell
                    }
                }
            }
        
            return deviation;
        },
        ontick(e){
            if(this._isActive && this.vehicle._t < 1){
                this.vehicle.drivePath(10);
            }
            if(this.vehicle._ds !== 0){
                this._tick++;
                const step = 2.5;
                const dsTick = this.vehicle._ds > 0 ? 
                               this.vehicle._ds > step ? step : this.vehicle._ds : 
                               this.vehicle._ds < -step ? -step : this.vehicle._ds;
                //this.vehicle.incrementPhi(e.dt);
                this.vehicle.A.iadd(this.vehicle.u.scl(dsTick));
                const dir = Math.sign(dsTick)
                this.notify(e.dt, dir)
                Math.abs(this.vehicle._ds) <= step ? this.vehicle._ds = 0 : this.vehicle._ds -= dsTick;
                if(this._tick === 60) {
                    this._tick = 0;
                    this._history.push(this.trailers.map(trailer => trailer.getDimensions()));
                }
            }
            this.vehicle.correctHandle();
            this._g.exe(this._selector).exe(this._ctx);
        },
        ondrag(e){// only modify selected geometry here .. do not redraw .. !
            if (this._selector.selection && this._selector.selection.drag) {
                const ds = v2({x:e.dxusr, y: e.dyusr});
                this._selector.selection.p.iadd(ds);

                if(this._selector.selection.label === 'Z' ){
                    this.vehicle.handle.add(this.vehicle.A.neg).r < 45 ? this.vehicle._ds -= ds.r : this.vehicle.handle.add(this.vehicle.A.neg).r > 55 ? this.vehicle._ds += ds.r : null;
                    this.vehicle.u = () => this.vehicle.A.neg.add(this.vehicle.handle).unit;
                }
            }

        },
        onpan(e){ 
            this._interactor.view.x += e.dx; 
            this._interactor.view.y += e.dy;
        },
        onwheel(e){
            e.dscl > 1 && this._interactor.view.scl > 0.1 ? this._interactor.view.scl -= 0.1: this._interactor.view.scl += 0.1;
        },
        toRad(deg){ return deg === 0 ? 0.001*Math.PI/180 : deg*Math.PI/180},
        round(num,digits){ return Math.floor(num * 10**digits)/10**digits;},
        notify(dt,dir){ for(const trailer of this.trailers) {
            trailer._dt = dt;
            trailer.dir = dir;
            trailer.update(dt);
        }},
        update(){this.vehicle.update()},
        use(g){
            const commands = this._g.commands.slice(0,3); 
                commands.push( g2().use({grp: g}).commands[0] );
            this._g.commands = commands.concat( this._g.commands.slice(3) );
        }
    }
}

const ctrlUpdate = () => document.getElementById('ctrl').update(); //ctrl-ing API for manual update
