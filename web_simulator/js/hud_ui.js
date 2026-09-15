(function () {
  const hud = document.getElementById('hud');
  const header = document.getElementById('hud-header');
  const collapseBtn = document.getElementById('hud-collapse-btn');
  const tabBtns = Array.from(document.querySelectorAll('.hud-tab-btn'));
  const tabPanels = Array.from(document.querySelectorAll('.hud-tab-panel'));

  function loadState() {
    try {
      return JSON.parse(localStorage.getItem('hudUiState') || '{}');
    } catch (e) {
      return {};
    }
  }
  function saveState(state) {
    try {
      localStorage.setItem('hudUiState', JSON.stringify(state));
    } catch (e) {
      // localStorage unavailable (private mode, etc.) - layout just won't persist.
    }
  }

  const state = loadState();

  function clampPosition(left, top) {
    // Clamp against the HUD's actual current size so it can never be dragged
    // (or resized via tab switch) partially or fully off-screen.
    const width = hud.offsetWidth || 280;
    const height = hud.offsetHeight || 44;
    const maxLeft = Math.max(0, window.innerWidth - width);
    const maxTop = Math.max(0, window.innerHeight - height);
    return { left: Math.min(Math.max(left, 0), maxLeft), top: Math.min(Math.max(top, 0), maxTop) };
  }

  if (typeof state.left === 'number' && typeof state.top === 'number') {
    const p = clampPosition(state.left, state.top);
    hud.style.left = p.left + 'px';
    hud.style.top = p.top + 'px';
  }

  function reclampToViewport() {
    const rect = hud.getBoundingClientRect();
    const p = clampPosition(rect.left, rect.top);
    hud.style.left = p.left + 'px';
    hud.style.top = p.top + 'px';
  }

  function setCollapsed(collapsed) {
    hud.classList.toggle('collapsed', collapsed);
    collapseBtn.textContent = collapsed ? '▸' : '▾';
    collapseBtn.title = collapsed ? '展開' : '折りたたみ';
    state.collapsed = collapsed;
    saveState(state);
    reclampToViewport();
  }
  setCollapsed(!!state.collapsed);
  collapseBtn.addEventListener('click', () => setCollapsed(!hud.classList.contains('collapsed')));

  function setActiveTab(name) {
    tabBtns.forEach((btn) => btn.classList.toggle('active', btn.dataset.tab === name));
    tabPanels.forEach((panel) => panel.classList.toggle('active', panel.dataset.tabPanel === name));
    state.tab = name;
    saveState(state);
    reclampToViewport();
  }
  tabBtns.forEach((btn) => btn.addEventListener('click', () => setActiveTab(btn.dataset.tab)));
  const initialTab = tabBtns.some((b) => b.dataset.tab === state.tab) ? state.tab : tabBtns[0].dataset.tab;
  setActiveTab(initialTab);

  let dragging = false;
  let dragOffsetX = 0;
  let dragOffsetY = 0;

  function pointOf(e) {
    return e.touches && e.touches.length ? e.touches[0] : e;
  }

  function onPointerDown(e) {
    if (e.target.closest('button')) return;
    dragging = true;
    const rect = hud.getBoundingClientRect();
    const point = pointOf(e);
    dragOffsetX = point.clientX - rect.left;
    dragOffsetY = point.clientY - rect.top;
    hud.style.right = 'auto';
    document.body.style.userSelect = 'none';
  }
  function onPointerMove(e) {
    if (!dragging) return;
    const point = pointOf(e);
    const p = clampPosition(point.clientX - dragOffsetX, point.clientY - dragOffsetY);
    hud.style.left = p.left + 'px';
    hud.style.top = p.top + 'px';
  }
  function onPointerUp() {
    if (!dragging) return;
    dragging = false;
    document.body.style.userSelect = '';
    const rect = hud.getBoundingClientRect();
    state.left = rect.left;
    state.top = rect.top;
    saveState(state);
  }

  header.addEventListener('mousedown', onPointerDown);
  window.addEventListener('mousemove', onPointerMove);
  window.addEventListener('mouseup', onPointerUp);
  header.addEventListener('touchstart', onPointerDown, { passive: true });
  window.addEventListener('touchmove', onPointerMove, { passive: true });
  window.addEventListener('touchend', onPointerUp);

  window.addEventListener('resize', reclampToViewport);
})();
