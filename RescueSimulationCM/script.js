(() => {
    const SPEED = 800,
        DEF = { x: 0, y: 0, w: 800, h: 800 },
        svgCon = document.getElementById('svg-container'),
        btn = document.getElementById('generate'),
        resetBtn = document.getElementById('reset-zoom');

    let svg, view, panning = false, start = {}, base = {};

    // Apply viewBox and toggle reset-button state
    const update = () => {
        svg.setAttribute('viewBox',
            `${view.x} ${view.y} ${view.w} ${view.h}`
        );
        resetBtn.disabled =
            view.x === DEF.x && view.y === DEF.y &&
            view.w === DEF.w && view.h === DEF.h;
    };

    // Zoom in/out around mouse pointer
    const onWheel = e => {
        e.preventDefault();
        const r = svg.getBoundingClientRect(),
            scale = Math.exp(-e.deltaY * .002),
            // map screen coords to SVG coords
            mx = (e.clientX - r.left) / r.width * view.w + view.x,
            my = (e.clientY - r.top) / r.height * view.h + view.y;

        // adjust viewBox so (mx,my) stays fixed
        view.x += (mx - view.x) * (1 - scale);
        view.y += (my - view.y) * (1 - scale);
        view.w *= scale;
        view.h *= scale;
        update();
    };

    // Reset to default viewBox
    const resetView = () => {
        view = { ...DEF };
        update();
    };

    // Begin pan: record pointer pos & view state, change cursor, capture pointer
    const onDown = e => {
        if (e.button) return;               // only left button
        panning = true;
        start = { x: e.clientX, y: e.clientY };
        base = { ...view };
        svg.style.cursor = 'grabbing';
        svg.setPointerCapture?.(e.pointerId);
    };

    // Continue pan: translate viewBox by pointer movement
    const onMove = e => {
        if (!panning) return;
        const dx = (e.clientX - start.x) * view.w / svg.clientWidth,
            dy = (e.clientY - start.y) * view.h / svg.clientHeight;
        view.x = base.x - dx;
        view.y = base.y - dy;
        update();
    };

    // End pan: release capture & reset cursor
    const onUp = e => {
        if (!panning) return;
        panning = false;
        svg.style.cursor = '';
        svg.releasePointerCapture?.(e.pointerId);
    };

    // Animate each .route path in sequence
    const animate = () => {
        let delay = 0;
        svgCon.querySelectorAll('.route').forEach(path => {
            const L = path.getTotalLength(),
                T = L / SPEED;
            // prepare stroke-dash for “draw” effect
            Object.assign(path.style, {
                transition: 'none',
                strokeDasharray: L,
                strokeDashoffset: L
            });
            path.getBoundingClientRect(); // force layout
            // animate strokeDashoffset → 0
            path.style.transition =
                `stroke-dashoffset ${T}s linear ${delay}s`;
            path.style.strokeDashoffset = '0';
            delay += T;
        });
    };

    // Initialize whenever #svg-maze appears
    const init = () => {
        svg = document.getElementById('svg-maze');
        if (!svg) return;
        view = { ...DEF };
        update();
        svg.addEventListener('wheel', onWheel, { passive: false });
        svg.addEventListener('dblclick', resetView);
        svg.addEventListener('pointerdown', onDown);
        svg.addEventListener('pointermove', onMove);
        svg.addEventListener('pointerup', onUp);
    };

    document.addEventListener('DOMContentLoaded', init);
    new MutationObserver(init).observe(svgCon, { childList: true });

    btn.addEventListener('click', () => setTimeout(animate, 50));
    resetBtn.addEventListener('click', resetView);
})();