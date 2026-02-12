class AreaSelector {
  constructor(canvasId, options) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');
    this.modes = options.modes;

    // Build colors from modes, with fallback to defaults
    this.colors = {
      aoi: 'rgba(0,255,0,0.4)',
      min: 'rgba(255,214,0,0.4)',
      max: 'rgba(25,118,210,0.4)'
    };
    for (const m of this.modes) {
      if (m.color) this.colors[m.key] = m.color;
    }

    this.handleColor = "#ff8888";
    this.currentMode = this.modes[0].key;
    this.areas = {};
    for (const m of this.modes) this.areas[m.key] = null;
    this.show = false;
    this.activeHandle = null;
    this.dragOffset = null;
    this.draggingBox = false;
    this.draggingHandle = false;
    this.startBox = null;
    this.startMouse = null;
    this.onchange = options.onchange || null;

    this.minWidth = options.minWidth || 100;   // normalized units [0,1000]
    this.minHeight = options.minHeight || 100; // normalized units [0,1000]

    this._bindEvents();
    this.redraw();
  }

  _bindEvents() {
    this.canvas.addEventListener('mousedown', (e) => this._onMouseDown(e));
    this.canvas.addEventListener('mousemove', (e) => this._onMouseMove(e));
    window.addEventListener('mouseup', (e) => this._onMouseUp(e));
  }

  setMode(key) {
    this.currentMode = key;
    this.show = true;
    this.redraw();
  }

  setAreas(obj) {
    for (let k in obj) this.areas[k] = obj[k] ? {...obj[k]} : null;
    this.redraw();
  }

  getAreas() {
    let o = {};
    for (const mode of this.modes) o[mode.key] = this.areas[mode.key] ? {...this.areas[mode.key]} : null;
    return o;
  }

  hideAreas() {
    this.show = false;
    this.redraw();
  }

  redraw() {
    this.ctx.clearRect(0,0,this.canvas.width,this.canvas.height);
    if (this.show && this.areas[this.currentMode]) {
      this._drawBox(this.areas[this.currentMode], this.colors[this.currentMode]);
      this._drawHandles(this.areas[this.currentMode]);
    }
  }

  _drawBox(area, color) {
    const c = this._areaToCanvas(area);
    this.ctx.save();
    this.ctx.globalAlpha = 1.0;
    this.ctx.lineWidth = 2;
    this.ctx.strokeStyle = color;
    this.ctx.fillStyle = color;
    this.ctx.fillRect(c.x, c.y, c.w, c.h);
    this.ctx.strokeRect(c.x, c.y, c.w, c.h);
    this.ctx.restore();
  }

  _drawHandles(area) {
    const hs = this._getHandles(area);
    hs.forEach((h, i) => {
      this.ctx.save();
      this.ctx.fillStyle = this.handleColor;
      this.ctx.fillRect(h.x-5,h.y-5,10,10);
      this.ctx.restore();
    });
  }

  _getHandles(area) {
    const c = this._areaToCanvas(area);
    const {x, y, w, h} = c;
    return [
      {x: x,      y: y},        // NW
      {x: x+w/2,  y: y},        // N
      {x: x+w,    y: y},        // NE
      {x: x+w,    y: y+h/2},    // E
      {x: x+w,    y: y+h},      // SE
      {x: x+w/2,  y: y+h},      // S
      {x: x,      y: y+h},      // SW
      {x: x,      y: y+h/2}     // W
    ];
  }

  _areaToCanvas(area) {
    const {x1, y1, x2, y2} = area;
    const x = Math.round(x1 * this.canvas.width / 1000);
    const y = Math.round(y1 * this.canvas.height / 1000);
    const w = Math.round((x2-x1) * this.canvas.width / 1000);
    const h = Math.round((y2-y1) * this.canvas.height / 1000);
    return {x, y, w, h};
  }

  _canvasToArea(x, y, w, h) {
    const x1 = Math.round(x * 1000 / this.canvas.width);
    const y1 = Math.round(y * 1000 / this.canvas.height);
    const x2 = Math.round((x+w) * 1000 / this.canvas.width);
    const y2 = Math.round((y+h) * 1000 / this.canvas.height);
    return {x1, y1, x2, y2};
  }

  _onMouseDown(e) {
    if (!this.show || !this.areas[this.currentMode]) return;
    const area = this.areas[this.currentMode];
    const handles = this._getHandles(area);
    const mouse = this._mouseToCanvas(e);
    for (let i = 0; i < handles.length; i++) {
      const h = handles[i];
      if (Math.abs(mouse.x-h.x)<8 && Math.abs(mouse.y-h.y)<8) {
        this.activeHandle = i;
        this.draggingHandle = true;
        this.startBox = this._areaToCanvas(area);
        this.startMouse = mouse;
        return;
      }
    }
    // Move mode
    const c = this._areaToCanvas(area);
    if (mouse.x>c.x && mouse.x<c.x+c.w && mouse.y>c.y && mouse.y<c.y+c.h) {
      this.draggingBox = true;
      this.dragOffset = {dx: mouse.x-c.x, dy: mouse.y-c.y};
      this.startBox = c;
      this.startMouse = mouse;
    }
  }

  _onMouseMove(e) {
    if (!this.show || !this.areas[this.currentMode]) {
      this.canvas.style.cursor = 'default';
      return;
    }
    const area = this.areas[this.currentMode];
    const mouseRaw = this._mouseToCanvas(e);
    const mouse = {
      x: Math.max(0, Math.min(mouseRaw.x, this.canvas.width)),
      y: Math.max(0, Math.min(mouseRaw.y, this.canvas.height))
    };

    // Cursor management
    let hoverHandle = null;
    const handles = this._getHandles(area);
    for (let i = 0; i < handles.length; i++) {
      const h = handles[i];
      if (Math.abs(mouse.x-h.x)<8 && Math.abs(mouse.y-h.y)<8) {
        hoverHandle = i;
        break;
      }
    }
    // Pointer shape logic
    if (hoverHandle===0 || hoverHandle===4)   this.canvas.style.cursor = 'nwse-resize';
    else if (hoverHandle===2 || hoverHandle===6) this.canvas.style.cursor = 'nesw-resize';
    else if (hoverHandle===1 || hoverHandle===5) this.canvas.style.cursor = 'ns-resize';
    else if (hoverHandle===3 || hoverHandle===7) this.canvas.style.cursor = 'ew-resize';
    else if (hoverHandle == null) {
      const c = this._areaToCanvas(area);
      if (mouse.x>c.x && mouse.x<c.x+c.w && mouse.y>c.y && mouse.y<c.y+c.h) {
        this.canvas.style.cursor = 'move';
      } else {
        this.canvas.style.cursor = 'default';
      }
    }

    // Minimum size in canvas pixels
    const minW = this.minWidth * this.canvas.width / 1000;
    const minH = this.minHeight * this.canvas.height / 1000;

    if (this.draggingHandle && this.activeHandle!==null && this.startBox && this.startMouse) {
      let c = Object.assign({}, this.startBox);
      let diffX = mouse.x - this.startMouse.x;
      let diffY = mouse.y - this.startMouse.y;
      let x = c.x, y = c.y, w = c.w, h = c.h;

      switch(this.activeHandle) {
        case 0: x += diffX; y += diffY; w -= diffX; h -= diffY; break; // NW
        case 1: y += diffY; h -= diffY; break; // N
        case 2: w += diffX; y += diffY; h -= diffY; break; // NE
        case 3: w += diffX; break; // E
        case 4: w += diffX; h += diffY; break; // SE
        case 5: h += diffY; break; // S
        case 6: x += diffX; w -= diffX; h += diffY; break; // SW
        case 7: x += diffX; w -= diffX; break; // W
      }
      // Edge and minimum size
      x = Math.max(0, Math.min(x, this.canvas.width-minW));
      y = Math.max(0, Math.min(y, this.canvas.height-minH));
      w = Math.max(minW, Math.min(w, this.canvas.width-x));
      h = Math.max(minH, Math.min(h, this.canvas.height-y));

      let newArea = this._canvasToArea(x, y, w, h);
      this.areas[this.currentMode] = newArea;
      this.redraw();
      if (this.onchange) this.onchange(this.getAreas());
    }
    else if (this.draggingBox && this.startBox && this.startMouse) {
      let dx = mouse.x - this.startMouse.x;
      let dy = mouse.y - this.startMouse.y;
      let newX = Math.max(0, Math.min(this.startBox.x+dx, this.canvas.width-this.startBox.w));
      let newY = Math.max(0, Math.min(this.startBox.y+dy, this.canvas.height-this.startBox.h));
      let w = Math.max(minW, this.startBox.w);
      let h = Math.max(minH, this.startBox.h);
      let newArea = this._canvasToArea(newX, newY, w, h);
      this.areas[this.currentMode] = newArea;
      this.redraw();
      if (this.onchange) this.onchange(this.getAreas());
    }
  }

  _onMouseUp(e) {
    this.draggingHandle = false;
    this.draggingBox = false;
    this.activeHandle = null;
    this.dragOffset = null;
    this.startBox = null;
    this.startMouse = null;
  }

  _mouseToCanvas(e) {
    const rect = this.canvas.getBoundingClientRect();
    return {
      x: Math.round((e.clientX-rect.left)*(this.canvas.width/rect.width)),
      y: Math.round((e.clientY-rect.top)*(this.canvas.height/rect.height))
    };
  }
}
